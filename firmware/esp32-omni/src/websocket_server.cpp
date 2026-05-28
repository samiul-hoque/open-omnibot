#include "websocket_server.h"
#include "config.h"
#include "sensors.h"
#include "motors.h"
#include "motor_calibration.h"
#include "odometry.h"
#include "trajectory.h"
#include "openloop_executor.h"
#include "self_test.h"
#include "pid_controller.h"
#include <WiFi.h>
#include <esp_wifi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include <Update.h>
#include <time.h>
#include <esp_sntp.h>
#include <stdarg.h>
#include <map>
#include <string>

// ============================================
// Private Variables
// ============================================

static AsyncWebServer server(80);
static AsyncWebSocket ws("/ws");

// Per-client inbound message accumulator.
//
// AsyncWebSocket delivers a single WebSocket text message across one
// or more WS_EVT_DATA callbacks. Two independent things can split it:
//   (a) a single frame is bigger than the TCP receive buffer, so the
//       library calls us multiple times per frame (info->index grows
//       from 0 up to info->len in chunk-sized pieces).
//   (b) the message spans multiple FRAMES using WebSocket continuations
//       (info->final=false on all but the last frame).
// We need to buffer chunks until both (a) and (b) complete, then parse
// the reassembled JSON.
//
// Before 2026-04-19 the handler rejected anything that didn't arrive in
// a single chunk+frame, silently dropping large load_trajectory
// messages (>~1.5 KB JSON — e.g. circle_0_5m_strafe with 36 segments).
// The drop log went to Serial only, which made the bug invisible when
// flashed via OTA. Every other message type happened to fit in one
// chunk, so nothing else seemed broken.
//
// Keyed by client->id() so simultaneous large messages from multiple
// clients (rare but possible) don't interleave. Cleared on disconnect.
static std::map<uint32_t, std::string> g_wsRxBuffers;

// Cap on reassembled message size to bound memory. Any single message
// larger than this is dropped (with a wsLog so it's visible over the
// wire). 16 KiB is 10× the largest legitimate message we send today
// and still a small fraction of ESP32 heap.
static constexpr size_t MAX_WS_MESSAGE_BYTES = 16 * 1024;

// Last received velocity command
// NOTE: These are accessed from both WebSocket callback (async) and main loop.
// Volatile gives visibility but not atomicity across the three fields, so
// composite reads/writes are wrapped in cmdMux to keep the (vx, vy, omega)
// triple consistent.
static volatile float cmdVx = 0;
static volatile float cmdVy = 0;
static volatile float cmdOmega = 0;
static volatile uint32_t lastCommandTime = 0;
static portMUX_TYPE cmdMux = portMUX_INITIALIZER_UNLOCKED;

// Sanitize one velocity component: reject NaN/Inf, clamp to ±limit.
static inline float sanitizeVel(float v, float limit) {
    if (isnan(v) || isinf(v)) return 0.0f;
    if (v >  limit) return  limit;
    if (v < -limit) return -limit;
    return v;
}

// Calibration mode — prevents PID loop from overwriting direct PWM commands
static volatile bool calibrationMode = false;
static volatile uint32_t lastCalibrationTime = 0;

// Heading-hold feature flag + tuning. Off by default; toggled via
// {"type":"set_heading_hold","enabled":true,"gain":1.0,"deadzone":0.03,"alpha":0.3}.
// Protected by headingHoldMux for atomic read/write on dual-core ESP32.
static bool  headingHoldEnabled  = false;
static float headingHoldGain     = HEADING_HOLD_GAIN_DEFAULT;
static float headingHoldDeadzone = HEADING_HOLD_DEADZONE_DEFAULT;
static float headingHoldAlpha    = HEADING_HOLD_LPF_ALPHA_DEFAULT;
static portMUX_TYPE headingHoldMux = portMUX_INITIALIZER_UNLOCKED;

// Debug broadcast mode — appends PID internals to sensor messages
static volatile bool  debugModeEnabled = false;
static volatile uint8_t debugRateDivider = 10;  // every Nth broadcast
static volatile uint8_t debugTickCounter = 0;

// WiFi AP mode tracking
static bool wifiApMode = false;

// NVS preferences for IMU calibration persistence
static Preferences preferences;

// NTP time sync state
static volatile bool ntpSynced = false;

// HTTP OTA reboot flag
static bool httpOtaReboot = false;

static void ntpSyncCallback(struct timeval* tv) {
    ntpSynced = true;
    Serial.println("NTP time synchronized");
}

// Get current UTC time in milliseconds, or 0 if not synced
static int64_t getUTCMillis() {
    if (!ntpSynced) return 0;
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (int64_t)tv.tv_sec * 1000LL + tv.tv_usec / 1000LL;
}

// ============================================
// Wireless Debug Logging
// ============================================

void wsLog(const char* format, ...) {
    if (ws.count() == 0) return;

    char msg[256];
    va_list args;
    va_start(args, format);
    vsnprintf(msg, sizeof(msg), format, args);
    va_end(args);

    JsonDocument doc;
    doc["type"] = "log";
    doc["msg"] = msg;

    String output;
    serializeJson(doc, output);
    ws.textAll(output);
}

// Broadcast raw JSON string to all connected WebSocket clients
void wsBroadcastRaw(const char* json) {
    ws.textAll(json);
}

// ============================================
// WebSocket Event Handler
// ============================================

static void onWebSocketEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                             AwsEventType type, void* arg, uint8_t* data, size_t len) {
    switch (type) {
        case WS_EVT_CONNECT:
            wsLog("WebSocket client #%u connected from %s",
                  client->id(), client->remoteIP().toString().c_str());
            break;
            
        case WS_EVT_DISCONNECT:
            wsLog("WebSocket client #%u disconnected", client->id());
            // Free any partial inbound message buffer so a client that
            // dropped mid-send doesn't leak RAM.
            g_wsRxBuffers.erase(client->id());
            // Stop motors when client disconnects.
            // Wrap the (vx, vy, omega) zero-out in cmdMux so the main loop
            // can't torn-read in the middle.
            portENTER_CRITICAL(&cmdMux);
            cmdVx = 0;
            cmdVy = 0;
            cmdOmega = 0;
            portEXIT_CRITICAL(&cmdMux);
            stopAllMotors();
            calibrationMode = false;
            if (isMotorCalibrationRunning()) {
                abortMotorCalibration();
            }
            if (trajGetState() != TRAJ_IDLE) {
                trajAbort();
            }
            if (isSelfTestRunning()) {
                abortSelfTest();
            }
            break;
            
        case WS_EVT_DATA: {
            AwsFrameInfo* info = (AwsFrameInfo*)arg;
            if (info->opcode != WS_TEXT) {
                // Binary / ping / pong — not used by our protocol. Ignore.
                return;
            }

            // Message-boundary accounting. Three cases for the incoming
            // chunk:
            //
            //   a) info->index == 0  → a new message starts. Ensure an
            //      empty buffer entry for this client (create if missing,
            //      clear if already present from a previous message that
            //      got interrupted before it completed — on the happy
            //      path the post-dispatch block below erases the entry,
            //      so the "clear" branch is only exercised after an
            //      aborted/partial predecessor).
            //
            //   b) info->index > 0 and we have a buffer → mid-message
            //      chunk on the happy path; append to the existing buffer.
            //
            //   c) info->index > 0 and we don't have a buffer → an
            //      earlier chunk of *this same message* was rejected (size
            //      cap, see below). Drop every tail chunk silently until
            //      the next info->index == 0 re-arms us. Without this
            //      guard, `g_wsRxBuffers[cid]` would create a fresh empty
            //      entry and the tail chunks would append into it,
            //      producing a "completed" message made entirely of mid-
            //      and end-chunks of the rejected one — which later fails
            //      JSON parse and emits a confusing error log. Per-client
            //      state (the map entry's presence) is the recovery flag.
            const uint32_t cid = client->id();
            auto it = g_wsRxBuffers.find(cid);
            if (info->index == 0) {
                if (it == g_wsRxBuffers.end()) {
                    it = g_wsRxBuffers.emplace(cid, std::string()).first;
                } else {
                    it->second.clear();
                }
            } else if (it == g_wsRxBuffers.end()) {
                return;  // case (c) — dropped-message tail
            }
            auto& buf = it->second;

            // Size guard: reject runaway messages before we let them
            // pressure the heap.
            //
            // Erase BEFORE wsLog. wsLog queues an outgoing broadcast,
            // and in a pathological TCP-send-failure path the library
            // can synchronously fire WS_EVT_DISCONNECT for this same
            // client, whose handler also calls
            // `g_wsRxBuffers.erase(client->id())`. If we'd called wsLog
            // first, our `auto& buf` reference above would be left
            // dangling when DISCONNECT's erase ran inside wsLog — any
            // code later in this branch that touched `buf` would be
            // UAF. We return immediately today so there's no actual
            // use-after-free at runtime, but doing the erase first
            // removes the footgun for future edits and makes
            // DISCONNECT's own erase a no-op (entry already gone).
            if (buf.size() + len > MAX_WS_MESSAGE_BYTES) {
                g_wsRxBuffers.erase(it);  // invalidates `it` and `buf`
                wsLog("WS: message too large (>%u bytes), dropping",
                      (unsigned)MAX_WS_MESSAGE_BYTES);
                return;
            }

            buf.append(reinterpret_cast<const char*>(data), len);

            // Two separate "are we done?" conditions, both required:
            //   (1) this chunk completes the current frame
            //   (2) this frame is the final frame of the message
            // info->index is the byte offset of this chunk within the
            // current frame; info->len is that frame's total length.
            const bool frameComplete = (info->index + len) == info->len;
            if (!frameComplete) return;          // more chunks of this frame
            if (!info->final) return;            // continuation frames follow

            {
                // Reassembled message — parse JSON.
                //
                // ArduinoJson v7 (bblanchon/ArduinoJson@7.x) with
                // `deserializeJson(doc, const char*, size_t)` COPIES all
                // string values into the JsonDocument's own ResourceManager
                // (as TinyString inline in VariantData, or as OwnedString
                // on the document's heap). `LinkedString` — the only
                // variant that would keep a pointer back into `buf` —
                // is never produced by this overload; it's reserved for
                // the explicit `F()`/flash-string APIs. See
                // `.pio/libdeps/*/ArduinoJson/src/ArduinoJson/Json/
                // JsonDeserializer.hpp::parseStringValue` and
                // `Memory/StringBuilder.hpp::save`. Every handler below
                // that reads `doc["type"]`, `doc["runId"]`, `s["k"]` etc.
                // therefore gets a pointer into the doc, not into buf —
                // we're free to release buf immediately after a successful
                // parse and not carry its storage through the dispatch.
                JsonDocument doc;
                DeserializationError error = deserializeJson(doc, buf.c_str(), buf.size());
                // Release the buffer as soon as the parse is done (or
                // failed). The doc owns every string it exposes, so the
                // downstream handlers don't care that the map entry is
                // gone. Keeps per-client steady-state memory at ~0 bytes
                // when the line is idle rather than pinned at the
                // largest-ever-seen message size for the lifetime of
                // the connection.
                g_wsRxBuffers.erase(it);

                if (error) {
                    wsLog("JSON parse error: %s", error.c_str());
                    return;
                }

                const char* msgType = doc["type"];
                if (msgType == nullptr) return;

                if (strcmp(msgType, "cmd") == 0) {
                    // Velocity command - standard robotics convention
                    // vx = forward (m/s), vy = left (m/s), w = CCW (rad/s)
                    float vx = sanitizeVel(doc["vx"] | 0.0f, MAX_LINEAR_VEL_MPS);
                    float vy = sanitizeVel(doc["vy"] | 0.0f, MAX_LINEAR_VEL_MPS);
                    float w  = sanitizeVel(doc["w"]  | 0.0f, MAX_ANGULAR_VEL_RPS);
                    portENTER_CRITICAL(&cmdMux);
                    cmdVx = vx;
                    cmdVy = vy;
                    cmdOmega = w;
                    lastCommandTime = millis();
                    portEXIT_CRITICAL(&cmdMux);
                }
                else if (strcmp(msgType, "stop") == 0) {
                    // Emergency stop. cmdMux keeps the (vx, vy, omega) triple
                    // coherent against the main loop's getLastVelocityCommand().
                    portENTER_CRITICAL(&cmdMux);
                    cmdVx = 0;
                    cmdVy = 0;
                    cmdOmega = 0;
                    lastCommandTime = millis();
                    portEXIT_CRITICAL(&cmdMux);
                    stopAllMotors();
                    if (trajGetState() != TRAJ_IDLE) {
                        trajAbort();
                    }
                    if (isSelfTestRunning()) {
                        abortSelfTest();
                    }
                    wsLog("STOP command received");
                    
                    // Send acknowledgment
                    client->text("{\"type\":\"ack\",\"cmd\":\"stop\"}");
                }
                else if (strcmp(msgType, "reset_encoders") == 0) {
                    resetAllEncoders();
                    resetOdomEncoders();
                    wsLog("Encoders reset");
                    client->text("{\"type\":\"ack\",\"cmd\":\"reset_encoders\"}");
                }
                else if (strcmp(msgType, "zero_imu") == 0) {
                    zeroIMU();
                    // Reset odometry heading to 0 while preserving position,
                    // so the complementary filter re-anchors to the new IMU
                    // reference frame instead of fighting the old offset.
                    Pose p = odomGetPose();
                    resetOdometry(p.x, p.y, 0);
                    wsLog("IMU orientation zeroed + heading reset");
                    client->text("{\"type\":\"ack\",\"cmd\":\"zero_imu\"}");
                }
                else if (strcmp(msgType, "ping") == 0) {
                    // Respond to ping with pong
                    client->text("{\"type\":\"pong\"}");
                }
                else if (strcmp(msgType, "ping_cal") == 0) {
                    // Calibration ping — echo back all timestamps for per-hop measurement
                    JsonDocument pongDoc;
                    pongDoc["type"] = "pong_cal";
                    pongDoc["ts"] = doc["ts"];
                    pongDoc["ts_server_fwd"] = doc["ts_server_fwd"];
                    int64_t robotUtc = getUTCMillis();
                    if (robotUtc > 0) {
                        pongDoc["rt"] = robotUtc;
                    }
                    pongDoc["ntpSynced"] = ntpSynced;

                    String pongOutput;
                    serializeJson(pongDoc, pongOutput);
                    client->text(pongOutput);
                }
                else if (strcmp(msgType, "motor_test") == 0) {
                    // Direct motor control for calibration — bypasses PID.
                    //
                    // Dashboard sends the EXTERNAL wire-order index
                    // ([L1, R1, R2, L2]) because that matches the order
                    // used by the sensor broadcast (enc[] / vel[]).
                    // setMotorSpeed() takes the INTERNAL index
                    // ([L1, R1, L2, R2]), so positions 2 and 3 must be
                    // swapped. Without this, the dashboard's "R2" card
                    // spins the L2 wheel and vice versa.
                    static const int EXT_TO_INT[4] = {0, 1, 3, 2};

                    int motor = doc["motor"] | -1;
                    int pwm = doc["pwm"] | 0;
                    pwm = constrain(pwm, -255, 255);

                    calibrationMode = true;
                    lastCalibrationTime = millis();
                    lastCommandTime = millis();

                    if (motor >= 0 && motor <= 3) {
                        setMotorSpeed(EXT_TO_INT[motor], pwm);
                    } else if (motor == 4) {
                        // All motors — no permutation needed
                        for (int i = 0; i < 4; i++) {
                            setMotorSpeed(i, pwm);
                        }
                    }
                }
                else if (strcmp(msgType, "get_info") == 0) {
                    // Respond with robot system info
                    JsonDocument infoDoc;
                    infoDoc["type"] = "info";
                    infoDoc["firmware"] = FIRMWARE_VERSION;
                    infoDoc["uptime"] = millis();
                    infoDoc["ip"] = wifiApMode
                        ? WiFi.softAPIP().toString()
                        : WiFi.localIP().toString();
                    infoDoc["rssi"] = wifiApMode ? 0 : WiFi.RSSI();
                    infoDoc["ssid"] = wifiApMode ? AP_SSID : WiFi.SSID();
                    infoDoc["mac"] = WiFi.macAddress();
                    infoDoc["freeHeap"] = ESP.getFreeHeap();
                    infoDoc["apMode"] = wifiApMode;
                    infoDoc["imuAvailable"] = isIMUAvailable();
                    infoDoc["ntpSynced"] = ntpSynced;
                    int64_t utcNow = getUTCMillis();
                    if (utcNow > 0) {
                        infoDoc["ntpTime"] = utcNow;
                    }

                    // Include per-direction motor calibration gains
                    float gFwd[4], gRev[4];
                    getMotorGains(gFwd, gRev);
                    JsonArray fwdArr = infoDoc["motorGainsFwd"].to<JsonArray>();
                    JsonArray revArr = infoDoc["motorGainsRev"].to<JsonArray>();
                    for (int i = 0; i < 4; i++) {
                        fwdArr.add(serialized(String(gFwd[i], 4)));
                        revArr.add(serialized(String(gRev[i], 4)));
                    }

                    // Odometry state
                    Pose pose = odomGetPose();
                    JsonObject odom = infoDoc["odom"].to<JsonObject>();
                    odom["x"] = serialized(String(pose.x, 4));
                    odom["y"] = serialized(String(pose.y, 4));
                    odom["th"] = serialized(String(pose.theta, 4));
                    odom["imuWeight"] = serialized(String(odomGetImuWeight(), 3));

                    // Trajectory state — order must match TrajectoryState enum.
                    static const char* TRAJ_STATE_NAMES[] = {
                        "idle", "armed", "running", "paused", "completed",
                    };
                    infoDoc["trajState"] = TRAJ_STATE_NAMES[trajGetState()];
                    infoDoc["trajMode"] = (trajGetMode() == TRAJ_MODE_OPEN_LOOP) ?
                        "openloop" : "closedloop";

                    // Open-loop calibration status — lets the dashboard
                    // show whether tier-0 is usable. When valid=false,
                    // every load_trajectory with mode:openloop will fail
                    // fast at load time.
                    const OpenLoopCal& olc = openloopGetCal();
                    JsonObject ol = infoDoc["openloopCal"].to<JsonObject>();
                    ol["valid"] = olc.valid;
                    ol["basePwm"] = olc.basePwm;
                    JsonObject olSpeeds = ol["speeds"].to<JsonObject>();
                    olSpeeds["fwd"]      = olc.speed[OL_DIR_FWD];
                    olSpeeds["back"]     = olc.speed[OL_DIR_BACK];
                    olSpeeds["strafe_l"] = olc.speed[OL_DIR_STRAFE_L];
                    olSpeeds["strafe_r"] = olc.speed[OL_DIR_STRAFE_R];
                    olSpeeds["yaw_ccw"]  = olc.speed[OL_DIR_YAW_CCW];
                    olSpeeds["yaw_cw"]   = olc.speed[OL_DIR_YAW_CW];

                    // Debug mode
                    infoDoc["debugMode"] = (bool)debugModeEnabled;

                    String infoOutput;
                    serializeJson(infoDoc, infoOutput);
                    client->text(infoOutput);
                }
                else if (strcmp(msgType, "save_imu_cal") == 0) {
                    // Save BNO055 calibration offsets to NVS
                    bool success = false;
                    if (isIMUAvailable()) {
                        adafruit_bno055_offsets_t offsets;
                        if (getIMUCalibrationOffsets(offsets)) {
                            preferences.begin("imu_cal", false);
                            preferences.putBytes("offsets", &offsets, sizeof(offsets));
                            preferences.putBool("valid", true);
                            preferences.end();
                            success = true;
                            wsLog("IMU calibration saved to NVS");
                        }
                    }

                    JsonDocument ackDoc;
                    ackDoc["type"] = "ack";
                    ackDoc["cmd"] = "save_imu_cal";
                    ackDoc["success"] = success;
                    String ackOutput;
                    serializeJson(ackDoc, ackOutput);
                    client->text(ackOutput);
                }
                else if (strcmp(msgType, "load_imu_cal") == 0) {
                    // Load BNO055 calibration offsets from NVS
                    bool success = false;
                    if (isIMUAvailable()) {
                        preferences.begin("imu_cal", true);
                        bool valid = preferences.getBool("valid", false);
                        if (valid) {
                            adafruit_bno055_offsets_t offsets;
                            preferences.getBytes("offsets", &offsets, sizeof(offsets));
                            setIMUCalibrationOffsets(offsets);
                            success = true;
                            wsLog("IMU calibration loaded from NVS");
                        }
                        preferences.end();
                    }

                    JsonDocument ackDoc;
                    ackDoc["type"] = "ack";
                    ackDoc["cmd"] = "load_imu_cal";
                    ackDoc["success"] = success;
                    String ackOutput;
                    serializeJson(ackDoc, ackOutput);
                    client->text(ackOutput);
                }
                else if (strcmp(msgType, "save_motor_cal") == 0) {
                    // Save per-direction motor calibration gains to NVS
                    float gFwd[4], gRev[4];
                    getMotorGains(gFwd, gRev);
                    preferences.begin("motor_cal", false);
                    for (int i = 0; i < 4; i++) {
                        char kf[8], kr[8];
                        snprintf(kf, sizeof(kf), "gf_%d", i);
                        snprintf(kr, sizeof(kr), "gr_%d", i);
                        preferences.putFloat(kf, gFwd[i]);
                        preferences.putFloat(kr, gRev[i]);
                    }
                    preferences.putBool("valid", true);
                    preferences.end();
                    wsLog("Motor calibration saved to NVS");

                    JsonDocument ackDoc;
                    ackDoc["type"] = "ack";
                    ackDoc["cmd"] = "save_motor_cal";
                    ackDoc["success"] = true;
                    JsonArray fArr = ackDoc["gainsFwd"].to<JsonArray>();
                    JsonArray rArr = ackDoc["gainsRev"].to<JsonArray>();
                    for (int i = 0; i < 4; i++) {
                        fArr.add(serialized(String(gFwd[i], 4)));
                        rArr.add(serialized(String(gRev[i], 4)));
                    }
                    String ackOutput;
                    serializeJson(ackDoc, ackOutput);
                    client->text(ackOutput);
                }
                else if (strcmp(msgType, "load_motor_cal") == 0) {
                    // Load per-direction motor calibration gains from NVS
                    bool success = false;
                    preferences.begin("motor_cal", true);
                    bool valid = preferences.getBool("valid", false);
                    if (valid) {
                        float gFwd[4], gRev[4];
                        for (int i = 0; i < 4; i++) {
                            char kf[8], kr[8];
                            snprintf(kf, sizeof(kf), "gf_%d", i);
                            snprintf(kr, sizeof(kr), "gr_%d", i);
                            gFwd[i] = preferences.getFloat(kf, 1.0f);
                            gRev[i] = preferences.getFloat(kr, 1.0f);
                        }
                        setMotorGains(gFwd, gRev);
                        success = true;
                        wsLog("Motor calibration loaded from NVS");
                    }
                    preferences.end();

                    float gFwd[4], gRev[4];
                    getMotorGains(gFwd, gRev);
                    JsonDocument ackDoc;
                    ackDoc["type"] = "ack";
                    ackDoc["cmd"] = "load_motor_cal";
                    ackDoc["success"] = success;
                    JsonArray fArr = ackDoc["gainsFwd"].to<JsonArray>();
                    JsonArray rArr = ackDoc["gainsRev"].to<JsonArray>();
                    for (int i = 0; i < 4; i++) {
                        fArr.add(serialized(String(gFwd[i], 4)));
                        rArr.add(serialized(String(gRev[i], 4)));
                    }
                    String ackOutput;
                    serializeJson(ackDoc, ackOutput);
                    client->text(ackOutput);
                }
                else if (strcmp(msgType, "set_motor_gains") == 0) {
                    // Manually set per-direction motor calibration gains
                    JsonArray fArr = doc["gainsFwd"];
                    JsonArray rArr = doc["gainsRev"];
                    if (fArr && fArr.size() == 4 && rArr && rArr.size() == 4) {
                        float gFwd[4], gRev[4];
                        for (int i = 0; i < 4; i++) {
                            gFwd[i] = fArr[i].as<float>();
                            gRev[i] = rArr[i].as<float>();
                        }
                        setMotorGains(gFwd, gRev);
                        wsLog("Motor gains set manually");

                        JsonDocument ackDoc;
                        ackDoc["type"] = "ack";
                        ackDoc["cmd"] = "set_motor_gains";
                        ackDoc["success"] = true;
                        JsonArray af = ackDoc["gainsFwd"].to<JsonArray>();
                        JsonArray ar = ackDoc["gainsRev"].to<JsonArray>();
                        for (int i = 0; i < 4; i++) {
                            af.add(serialized(String(gFwd[i], 4)));
                            ar.add(serialized(String(gRev[i], 4)));
                        }
                        String ackOutput;
                        serializeJson(ackDoc, ackOutput);
                        client->text(ackOutput);
                    }
                }
                else if (strcmp(msgType, "set_openloop_cal") == 0) {
                    // Store tier-0 open-loop calibration to NVS. Expects:
                    //   { basePwm: int,
                    //     speeds: { fwd, back, strafe_l, strafe_r, yaw_ccw, yaw_cw } }
                    // All six speeds must be positive finite numbers;
                    // basePwm must be in (0, 255]. Partial cal is rejected
                    // — a half-populated table is worse than no cal at
                    // all because load_trajectory would then accept some
                    // segments and reject others unpredictably.
                    OpenLoopCal cal = {};
                    const int pwm = doc["basePwm"] | 0;
                    if (pwm <= 0 || pwm > 255) {
                        client->text("{\"type\":\"ack\",\"cmd\":\"set_openloop_cal\","
                                     "\"ok\":false,\"error\":\"bad_basePwm\"}");
                        break;
                    }
                    cal.basePwm = (uint8_t)pwm;
                    JsonObject sp = doc["speeds"];
                    if (!sp) {
                        client->text("{\"type\":\"ack\",\"cmd\":\"set_openloop_cal\","
                                     "\"ok\":false,\"error\":\"missing_speeds\"}");
                        break;
                    }
                    cal.speed[OL_DIR_FWD]      = sp["fwd"]      | 0.0f;
                    cal.speed[OL_DIR_BACK]     = sp["back"]     | 0.0f;
                    cal.speed[OL_DIR_STRAFE_L] = sp["strafe_l"] | 0.0f;
                    cal.speed[OL_DIR_STRAFE_R] = sp["strafe_r"] | 0.0f;
                    cal.speed[OL_DIR_YAW_CCW]  = sp["yaw_ccw"]  | 0.0f;
                    cal.speed[OL_DIR_YAW_CW]   = sp["yaw_cw"]   | 0.0f;
                    cal.valid = true;  // openloopSaveCal re-validates
                    if (openloopSaveCal(cal)) {
                        client->text("{\"type\":\"ack\",\"cmd\":\"set_openloop_cal\",\"ok\":true}");
                    } else {
                        client->text("{\"type\":\"ack\",\"cmd\":\"set_openloop_cal\","
                                     "\"ok\":false,\"error\":\"invalid_speeds\"}");
                    }
                }
                else if (strcmp(msgType, "start_motor_cal") == 0) {
                    // Start automated motor calibration routine. Refuse
                    // if a trajectory is in flight — the cal routine
                    // drives motors directly (bypassing the trajectory
                    // follower), so a concurrent TRAJ_RUNNING would
                    // fight the cal's PWM sweep and a TRAJ_PAUSED robot
                    // would silently start moving mid-experiment when
                    // the cal ramps up. Same guard that start_self_test
                    // already uses.
                    TrajectoryState ts = trajGetState();
                    if (ts == TRAJ_RUNNING || ts == TRAJ_PAUSED) {
                        wsLog("Motor-cal: blocked — trajectory is %s",
                              ts == TRAJ_RUNNING ? "running" : "paused");
                        client->text(
                            "{\"type\":\"ack\",\"cmd\":\"start_motor_cal\","
                            "\"ok\":false,\"error\":\"trajectory_active\"}");
                    } else {
                        calibrationMode = true;
                        lastCalibrationTime = millis();
                        startMotorCalibration();
                        client->text(
                            "{\"type\":\"ack\",\"cmd\":\"start_motor_cal\",\"ok\":true}");
                    }
                }
                else if (strcmp(msgType, "set_heading_hold") == 0) {
                    // Toggle IMU-based yaw-rate correction for pure
                    // translation. Optional fields override any of the P
                    // controller's tunables. Each is clamped to a sane
                    // range so a stray typo can't make the correction
                    // either inert or explosive.
                    bool en = doc["enabled"] | false;
                    float g = headingHoldGain, d = headingHoldDeadzone, a = headingHoldAlpha;
                    // ArduinoJson v7 idiom: `.is<T>()` is true only when
                    // the key exists AND the value is convertible to T.
                    // Preferred over the deprecated `containsKey` because
                    // it also rejects malformed values (e.g. "gain": "abc"
                    // would previously pass containsKey and then produce 0
                    // from `.as<float>()`).
                    if (doc["gain"].is<float>()) {
                        g = doc["gain"].as<float>();
                        if (g < 0.0f) g = 0.0f;
                        if (g > 5.0f) g = 5.0f;
                    }
                    if (doc["deadzone"].is<float>()) {
                        d = doc["deadzone"].as<float>();
                        if (d < 0.0f) d = 0.0f;
                        if (d > 1.0f) d = 1.0f;
                    }
                    if (doc["alpha"].is<float>()) {
                        a = doc["alpha"].as<float>();
                        if (a < 0.01f) a = 0.01f;   // any lower = dead filter
                        if (a > 1.0f)  a = 1.0f;    // 1.0 = pass-through
                    }
                    portENTER_CRITICAL(&headingHoldMux);
                    headingHoldEnabled = en;
                    headingHoldGain = g;
                    headingHoldDeadzone = d;
                    headingHoldAlpha = a;
                    portEXIT_CRITICAL(&headingHoldMux);
                    wsLog("Heading-hold: %s gain=%.2f deadzone=%.3f alpha=%.2f",
                          en ? "ENABLED" : "disabled", g, d, a);
                    JsonDocument ackDoc;
                    ackDoc["type"] = "ack";
                    ackDoc["cmd"] = "set_heading_hold";
                    ackDoc["enabled"] = (bool)headingHoldEnabled;
                    ackDoc["gain"] = headingHoldGain;
                    ackDoc["deadzone"] = headingHoldDeadzone;
                    ackDoc["alpha"] = headingHoldAlpha;
                    String ackOutput;
                    serializeJson(ackDoc, ackOutput);
                    client->text(ackOutput);
                }

                // ----- Odometry commands -----

                else if (strcmp(msgType, "reset_odom") == 0) {
                    float x = doc["x"] | 0.0f;
                    float y = doc["y"] | 0.0f;
                    float theta = doc["theta"] | 0.0f;
                    resetOdomEncoders();
                    resetOdometry(x, y, theta);
                    wsLog("Odometry reset to (%.3f, %.3f, %.3f)", x, y, theta);
                    client->text("{\"type\":\"ack\",\"cmd\":\"reset_odom\"}");
                }
                else if (strcmp(msgType, "set_odom_config") == 0) {
                    if (doc["imuWeight"].is<float>()) {
                        float w = doc["imuWeight"].as<float>();
                        odomSetImuWeight(w);
                        wsLog("Odometry IMU weight set to %.3f", odomGetImuWeight());
                    }
                    JsonDocument ackDoc;
                    ackDoc["type"] = "ack";
                    ackDoc["cmd"] = "set_odom_config";
                    ackDoc["imuWeight"] = odomGetImuWeight();
                    String ackStr;
                    serializeJson(ackDoc, ackStr);
                    client->text(ackStr);
                }

                // ----- Trajectory commands -----

                else if (strcmp(msgType, "load_trajectory") == 0) {
                    const char* id = doc["runId"] | "";
                    JsonArray segsArr = doc["segments"];
                    if (!segsArr || segsArr.size() == 0 ||
                        segsArr.size() > MAX_TRAJECTORY_SEGMENTS) {
                        wsLog("load_trajectory: invalid segments (count=%d)",
                              segsArr ? (int)segsArr.size() : -1);
                        client->text("{\"type\":\"ack\",\"cmd\":\"load_trajectory\",\"ok\":false}");
                        break;
                    }

                    TrajectorySegment parsed[MAX_TRAJECTORY_SEGMENTS];
                    int count = 0;
                    bool valid = true;

                    for (JsonObject s : segsArr) {
                        const char* k = s["k"] | "";
                        if (!k || !k[0]) {
                            wsLog("load_trajectory: segment %d has missing/empty kind", count);
                            valid = false;
                            break;
                        }
                        if (strcmp(k, "t") == 0) {
                            parsed[count].kind = SEG_TRANSLATE;
                            parsed[count].translate.vx = s["vx"] | 0.0f;
                            parsed[count].translate.vy = s["vy"] | 0.0f;
                            parsed[count].translate.distance = s["d"] | 0.0f;
                        } else if (strcmp(k, "y") == 0) {
                            parsed[count].kind = SEG_YAW;
                            parsed[count].yaw.w = s["w"] | 0.0f;
                            parsed[count].yaw.angle = s["a"] | 0.0f;
                        } else if (strcmp(k, "c") == 0) {
                            parsed[count].kind = SEG_STRAFE_CIRCLE;
                            parsed[count].circle.speed = s["s"] | 0.0f;
                            parsed[count].circle.radius = s["r"] | 0.0f;
                        } else if (strcmp(k, "p") == 0) {
                            // Pause marker — no payload. Firmware halts
                            // at this segment until `traj_resume` arrives.
                            parsed[count].kind = SEG_PAUSE;
                        } else {
                            wsLog("load_trajectory: unknown segment kind '%s'", k);
                            valid = false;
                            break;
                        }
                        count++;
                    }

                    // Optional `mode` field selects the execution path.
                    // "openloop" activates the tier-0 open-loop executor
                    // (see openloop_executor.cpp); anything else defaults
                    // to closed-loop (PID + IK), preserving the existing
                    // protocol for unmodified clients.
                    const char* modeStr = doc["mode"] | "";
                    const TrajMode mode = (strcmp(modeStr, "openloop") == 0)
                        ? TRAJ_MODE_OPEN_LOOP : TRAJ_MODE_CLOSED_LOOP;

                    if (valid && trajLoad(id, parsed, count, mode)) {
                        // Pre-run hygiene: reset encoders, odometry, IMU.
                        // Open-loop doesn't consult odometry/IMU for control,
                        // but we still zero them so the logged CSV starts
                        // clean and the watchdog baseline reads a known 0.
                        resetAllEncoders();
                        resetOdomEncoders();
                        resetOdometry();
                        zeroIMU();
                        trajArm();
                        client->text("{\"type\":\"ack\",\"cmd\":\"load_trajectory\",\"ok\":true}");
                    } else {
                        wsLog("load_trajectory: load failed (mode=%s)",
                              mode == TRAJ_MODE_OPEN_LOOP ? "openloop" : "closedloop");
                        client->text(
                            mode == TRAJ_MODE_OPEN_LOOP
                            ? "{\"type\":\"ack\",\"cmd\":\"load_trajectory\",\"ok\":false,"
                              "\"error\":\"openloop_load_failed\"}"
                            : "{\"type\":\"ack\",\"cmd\":\"load_trajectory\",\"ok\":false}");
                    }
                }
                else if (strcmp(msgType, "traj_start") == 0) {
                    if (trajGetState() == TRAJ_ARMED) {
                        trajStart();
                        client->text("{\"type\":\"ack\",\"cmd\":\"traj_start\",\"ok\":true}");
                    } else {
                        wsLog("traj_start: not armed (state=%d)", trajGetState());
                        client->text("{\"type\":\"ack\",\"cmd\":\"traj_start\",\"ok\":false}");
                    }
                }
                else if (strcmp(msgType, "traj_resume") == 0) {
                    if (trajGetState() == TRAJ_PAUSED) {
                        trajResume();
                        client->text("{\"type\":\"ack\",\"cmd\":\"traj_resume\",\"ok\":true}");
                    } else {
                        wsLog("traj_resume: not paused (state=%d)", trajGetState());
                        client->text("{\"type\":\"ack\",\"cmd\":\"traj_resume\",\"ok\":false}");
                    }
                }
                else if (strcmp(msgType, "traj_abort") == 0) {
                    trajAbort();
                    client->text("{\"type\":\"ack\",\"cmd\":\"traj_abort\",\"ok\":true}");
                }

                // ----- Self-test commands -----

                else if (strcmp(msgType, "start_self_test") == 0) {
                    if (isMotorCalibrationRunning() || trajGetState() == TRAJ_RUNNING) {
                        wsLog("Self-test: blocked — calibration or trajectory running");
                        client->text("{\"type\":\"ack\",\"cmd\":\"start_self_test\",\"ok\":false}");
                    } else {
                        calibrationMode = true;
                        lastCalibrationTime = millis();
                        if (startSelfTest(doc)) {
                            client->text("{\"type\":\"ack\",\"cmd\":\"start_self_test\",\"ok\":true}");
                        } else {
                            calibrationMode = false;
                            client->text("{\"type\":\"ack\",\"cmd\":\"start_self_test\",\"ok\":false}");
                        }
                    }
                }
                else if (strcmp(msgType, "abort_self_test") == 0) {
                    abortSelfTest();
                    calibrationMode = false;
                    client->text("{\"type\":\"ack\",\"cmd\":\"abort_self_test\",\"ok\":true}");
                }

                // ----- Debug mode -----

                else if (strcmp(msgType, "set_debug") == 0) {
                    debugModeEnabled = doc["enabled"] | false;
                    setPIDDiagEnabled(debugModeEnabled);
                    if (doc["rate_divider"].is<int>()) {
                        int rd = doc["rate_divider"] | 10;
                        if (rd < 1) rd = 1;
                        if (rd > 100) rd = 100;
                        debugRateDivider = (uint8_t)rd;
                    }
                    debugTickCounter = 0;
                    wsLog("Debug mode: %s (rate_divider=%d)",
                          debugModeEnabled ? "ENABLED" : "disabled", debugRateDivider);
                    JsonDocument ackDoc;
                    ackDoc["type"] = "ack";
                    ackDoc["cmd"] = "set_debug";
                    ackDoc["enabled"] = (bool)debugModeEnabled;
                    ackDoc["rate_divider"] = debugRateDivider;
                    String ackStr;
                    serializeJson(ackDoc, ackStr);
                    client->text(ackStr);
                }
            }
            break;
        }
            
        case WS_EVT_PONG:
            // Pong received
            break;
            
        case WS_EVT_ERROR:
            wsLog("WebSocket error for client #%u", client->id());
            break;
    }
}

// ============================================
// Initialization
// ============================================

void initWebSocket() {
    Serial.println("Connecting to WiFi...");
    
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println();
        Serial.println("WiFi connected!");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());

        // Disable WiFi power saving to prevent periodic radio sleep
        esp_wifi_set_ps(WIFI_PS_NONE);

        // Initialize SNTP for time synchronization
        sntp_set_time_sync_notification_cb(ntpSyncCallback);
        sntp_setoperatingmode(SNTP_OPMODE_POLL);
        sntp_setservername(0, NTP_SERVER1);
        sntp_setservername(1, NTP_SERVER2);
        sntp_init();
        Serial.println("SNTP initialized, waiting for time sync...");

        // Wait briefly for initial sync (non-blocking after timeout)
        uint32_t ntpStart = millis();
        while (!ntpSynced && (millis() - ntpStart) < NTP_SYNC_TIMEOUT_MS) {
            delay(100);
        }
        if (ntpSynced) {
            struct tm timeinfo;
            getLocalTime(&timeinfo, 0);
            Serial.printf("NTP synced: %04d-%02d-%02d %02d:%02d:%02d UTC\n",
                          timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                          timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
        } else {
            Serial.println("NTP sync pending (will sync in background)");
        }
    } else {
        Serial.println();
        Serial.println("WiFi failed, starting AP mode...");
        WiFi.softAP(AP_SSID, AP_PASSWORD);
        wifiApMode = true;
        Serial.print("AP IP: ");
        Serial.println(WiFi.softAPIP());
    }
    
    // Setup WebSocket
    ws.onEvent(onWebSocketEvent);
    server.addHandler(&ws);
    
    // Simple status page
    server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
        String html = "<!DOCTYPE html><html><head><title>Omni-2 Robot</title></head><body>";
        html += "<h1>Omni-2 Robot WebSocket Server</h1>";
        String ip = wifiApMode ? WiFi.softAPIP().toString() : WiFi.localIP().toString();
        html += "<p>WebSocket endpoint: ws://" + ip + "/ws</p>";
        html += "<p>Connected clients: " + String(ws.count()) + "</p>";
        html += "<p><a href=\"/update\">Firmware Update</a></p>";
        html += "</body></html>";
        request->send(200, "text/html", html);
    });

    // HTTP OTA: upload form
    server.on("/update", HTTP_GET, [](AsyncWebServerRequest* request) {
        String html = R"(<!DOCTYPE html><html><head><title>Omni-2 OTA</title>
<style>body{font-family:sans-serif;max-width:480px;margin:40px auto;padding:0 20px}
#p{display:none;background:#eee;border-radius:4px;overflow:hidden;margin:10px 0}
#b{width:0%;height:24px;background:#4CAF50;text-align:center;color:#fff;line-height:24px;transition:width .3s}
#s{margin:10px 0;font-weight:bold}</style></head><body>
<h2>Firmware Update</h2><p>Current: v)" FIRMWARE_VERSION R"(</p>
<form id="f"><input type="file" name="firmware" accept=".bin" required>
<button type="submit">Upload</button></form>
<div id="p"><div id="b">0%</div></div><div id="s"></div>
<script>document.getElementById('f').onsubmit=function(e){
e.preventDefault();var d=new FormData(this),x=new XMLHttpRequest(),b=document.getElementById('b'),
s=document.getElementById('s');document.getElementById('p').style.display='block';
x.upload.onprogress=function(e){if(e.lengthComputable){var p=Math.round(e.loaded/e.total*100);
b.style.width=p+'%';b.textContent=p+'%';}};
x.onload=function(){s.textContent=x.status===200?'Success! Rebooting...':'Update failed: '+x.responseText;};
x.onerror=function(){s.textContent='Connection lost';};
x.open('POST','/update');x.send(d);};</script></body></html>)";
        request->send(200, "text/html", html);
    });

    // HTTP OTA: firmware upload handler
    server.on("/update", HTTP_POST,
        [](AsyncWebServerRequest* request) {
            bool success = !Update.hasError();
            AsyncWebServerResponse* response = request->beginResponse(
                success ? 200 : 500, "text/plain",
                success ? "OK" : "Update failed");
            response->addHeader("Connection", "close");
            request->send(response);
            if (success) httpOtaReboot = true;
        },
        [](AsyncWebServerRequest* request, const String& filename,
           size_t index, uint8_t* data, size_t len, bool final) {
            if (index == 0) {
                Serial.printf("HTTP OTA: Update starting (%s)\n", filename.c_str());
                stopAllMotors();
                if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
                    Update.printError(Serial);
                }
            }
            if (Update.isRunning()) {
                if (Update.write(data, len) != len) {
                    Update.printError(Serial);
                }
            }
            if (final) {
                if (Update.end(true)) {
                    Serial.printf("HTTP OTA: Success (%u bytes)\n", index + len);
                } else {
                    Update.printError(Serial);
                }
            }
        }
    );

    server.begin();
    Serial.println("WebSocket server started on /ws");

    // Setup ArduinoOTA
    ArduinoOTA.setHostname("omni2");
    ArduinoOTA.onStart([]() {
        stopAllMotors();
        Serial.println("OTA: Update starting...");
    });
    ArduinoOTA.onEnd([]() {
        Serial.println("OTA: Update complete, rebooting...");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("OTA: %u%%\r", progress / (total / 100));
    });
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("OTA Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
    ArduinoOTA.begin();
    String otaIp = wifiApMode ? WiFi.softAPIP().toString() : WiFi.localIP().toString();
    Serial.printf("OTA ready (hostname: omni2, IP: %s)\n", otaIp.c_str());
}

// ============================================
// Runtime Functions
// ============================================

void handleWebSocket() {
    ws.cleanupClients(2);
    if (httpOtaReboot) {
        delay(100);
        ESP.restart();
    }

    // Periodic WiFi reconnection.
    // STA mode: retry every 10s if connection drops.
    // AP mode: try STA every 60s in case the network became available
    //          after the initial boot failure. On success, tear down AP.
    static uint32_t lastWifiCheck = 0;
    uint32_t now = millis();
    uint32_t interval = wifiApMode ? 60000 : 10000;
    if (now - lastWifiCheck > interval) {
        lastWifiCheck = now;
        if (!wifiApMode && WiFi.status() != WL_CONNECTED) {
            Serial.println("WiFi disconnected — attempting reconnect");
            WiFi.reconnect();
        } else if (wifiApMode) {
            // Try STA connection while keeping AP alive
            WiFi.mode(WIFI_AP_STA);
            WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
            // Non-blocking: check result on next interval
            if (WiFi.waitForConnectResult(5000) == WL_CONNECTED) {
                Serial.println("WiFi STA connected from AP mode — switching to STA");
                WiFi.softAPdisconnect(true);
                WiFi.mode(WIFI_STA);
                wifiApMode = false;
                esp_wifi_set_ps(WIFI_PS_NONE);
                Serial.print("STA IP: ");
                Serial.println(WiFi.localIP());
            } else {
                // Stay in AP mode
                WiFi.mode(WIFI_AP);
            }
        }
    }
}

void broadcastSensorData() {
    if (ws.count() == 0) return;  // No clients connected

    // Use cached sensor snapshot from the most recent 50Hz motor update
    // to avoid redundant I2C (IMU) and PCNT (encoder) reads.
    const CachedSensors& cached = getSensorCache();
    const EncoderData& enc = cached.enc;
    const IMUData& imu = cached.imu;
    Pose pose = odomGetPose();

    // IMU stuck-read watchdog (see sensors.cpp). Fire a one-shot wsLog
    // on the latch transition so the event appears in the log stream
    // too — the sticky `imuStuck` field in the sensor broadcast is what
    // the UI watches to raise a power-cycle banner.
    const bool imuStuck = isIMUStuck();
    static bool imuStuckPrev = false;
    if (imuStuck && !imuStuckPrev) {
        wsLog("IMU stuck — gz frozen; power-cycle the robot to recover");
    }
    imuStuckPrev = imuStuck;
    const char* imuStuckField = imuStuck ? ",\"imuStuck\":true" : "";

    // Hand-formatted JSON via snprintf into a static buffer. Previously this
    // allocated a JsonDocument + a String on every broadcast (~20 Hz), plus
    // a transient String per serialized float — all heap churn on the ESP32.
    // The message shape is stable so snprintf fits the job and allocates
    // nothing on the hot path.
    static char buf[1024];
    int n = 0;

    // Heap telemetry — cheap to read (register lookups, no I2C / syscall)
    // and surfaces fragmentation/leaks that would otherwise only be
    // visible by hand-inspection. `free` is the current heap free bytes;
    // `min` is the high-water-low mark since boot — if that trends
    // downward over long sessions it's a slow leak.
    const unsigned long heapFree = (unsigned long)ESP.getFreeHeap();
    const unsigned long heapMin  = (unsigned long)ESP.getMinFreeHeap();

    int64_t utcMs = getUTCMillis();
    if (utcMs > 0) {
        n = snprintf(buf, sizeof(buf),
            "{\"type\":\"sensors\",\"t\":%lu,\"utc\":%lld,"
            "\"enc\":[%ld,%ld,%ld,%ld],"
            "\"vel\":[%.3f,%.3f,%.3f,%.3f],"
            "\"imu\":{\"yaw\":%.2f,\"pitch\":%.2f,\"roll\":%.2f,"
            "\"gz\":%.3f,\"ax\":%.3f,\"ay\":%.3f},"
            "\"cal\":{\"sys\":%u,\"gyro\":%u,\"accel\":%u,\"mag\":%u},"
            "\"pose\":{\"x\":%.4f,\"y\":%.4f,\"th\":%.4f},"
            "\"heap\":{\"free\":%lu,\"min\":%lu}%s}",
            (unsigned long)enc.timestamp, (long long)utcMs,
            (long)enc.counts[0], (long)enc.counts[1], (long)enc.counts[3], (long)enc.counts[2],
            enc.velocities[0], enc.velocities[1], enc.velocities[3], enc.velocities[2],
            imu.yaw, imu.pitch, imu.roll, imu.gyro_z, imu.accel_x, imu.accel_y,
            imu.cal_system, imu.cal_gyro, imu.cal_accel, imu.cal_mag,
            pose.x, pose.y, pose.theta,
            heapFree, heapMin, imuStuckField);
    } else {
        n = snprintf(buf, sizeof(buf),
            "{\"type\":\"sensors\",\"t\":%lu,"
            "\"enc\":[%ld,%ld,%ld,%ld],"
            "\"vel\":[%.3f,%.3f,%.3f,%.3f],"
            "\"imu\":{\"yaw\":%.2f,\"pitch\":%.2f,\"roll\":%.2f,"
            "\"gz\":%.3f,\"ax\":%.3f,\"ay\":%.3f},"
            "\"cal\":{\"sys\":%u,\"gyro\":%u,\"accel\":%u,\"mag\":%u},"
            "\"pose\":{\"x\":%.4f,\"y\":%.4f,\"th\":%.4f},"
            "\"heap\":{\"free\":%lu,\"min\":%lu}%s}",
            (unsigned long)enc.timestamp,
            (long)enc.counts[0], (long)enc.counts[1], (long)enc.counts[3], (long)enc.counts[2],
            enc.velocities[0], enc.velocities[1], enc.velocities[3], enc.velocities[2],
            imu.yaw, imu.pitch, imu.roll, imu.gyro_z, imu.accel_x, imu.accel_y,
            imu.cal_system, imu.cal_gyro, imu.cal_accel, imu.cal_mag,
            pose.x, pose.y, pose.theta,
            heapFree, heapMin, imuStuckField);
    }

    // Optionally append PID debug diagnostics
    if (debugModeEnabled && n > 0 && n < (int)sizeof(buf)) {
        debugTickCounter++;
        if (debugTickCounter >= debugRateDivider) {
            debugTickCounter = 0;

            PIDDiag diag[4];
            getPIDDiagnostics(diag);
            VelocityCommand fk = mecanumForwardKinematics(
                enc.velocities[0], enc.velocities[1],
                enc.velocities[3], enc.velocities[2]);

            // Overwrite the closing '}' and append debug block
            n--;  // back up over '}'
            n += snprintf(buf + n, sizeof(buf) - n,
                ",\"dbg\":{\"pid\":["
                "{\"tgt\":%.3f,\"act\":%.3f,\"err\":%.3f,\"p\":%.2f,\"i\":%.2f,\"d\":%.2f,\"ff\":%.1f,\"pwm\":%d},"
                "{\"tgt\":%.3f,\"act\":%.3f,\"err\":%.3f,\"p\":%.2f,\"i\":%.2f,\"d\":%.2f,\"ff\":%.1f,\"pwm\":%d},"
                "{\"tgt\":%.3f,\"act\":%.3f,\"err\":%.3f,\"p\":%.2f,\"i\":%.2f,\"d\":%.2f,\"ff\":%.1f,\"pwm\":%d},"
                "{\"tgt\":%.3f,\"act\":%.3f,\"err\":%.3f,\"p\":%.2f,\"i\":%.2f,\"d\":%.2f,\"ff\":%.1f,\"pwm\":%d}],"
                "\"fk\":{\"vx\":%.3f,\"vy\":%.3f,\"w\":%.3f},"
                "\"heap\":%lu}}",
                diag[0].target, diag[0].actual, diag[0].error, diag[0].p_term, diag[0].i_term, diag[0].d_term, diag[0].feedforward, diag[0].pwm_out,
                diag[1].target, diag[1].actual, diag[1].error, diag[1].p_term, diag[1].i_term, diag[1].d_term, diag[1].feedforward, diag[1].pwm_out,
                diag[2].target, diag[2].actual, diag[2].error, diag[2].p_term, diag[2].i_term, diag[2].d_term, diag[2].feedforward, diag[2].pwm_out,
                diag[3].target, diag[3].actual, diag[3].error, diag[3].p_term, diag[3].i_term, diag[3].d_term, diag[3].feedforward, diag[3].pwm_out,
                fk.vx, fk.vy, fk.omega,
                (unsigned long)ESP.getFreeHeap());
        }
    }

    // snprintf returns the full length it *would* have written; guard against
    // accidental truncation silently sending malformed JSON.
    if (n > 0 && n < (int)sizeof(buf)) {
        ws.textAll(buf);
    } else {
        wsLog("broadcastSensorData: buffer too small (%d)", n);
    }
}

bool isClientConnected() {
    return ws.count() > 0;
}

void getLastVelocityCommand(float* vx, float* vy, float* omega) {
    portENTER_CRITICAL(&cmdMux);
    *vx = cmdVx;
    *vy = cmdVy;
    *omega = cmdOmega;
    portEXIT_CRITICAL(&cmdMux);
}

bool isVelocityCommandValid() {
    portENTER_CRITICAL(&cmdMux);
    uint32_t t = lastCommandTime;
    portEXIT_CRITICAL(&cmdMux);
    return (millis() - t) < VELOCITY_TIMEOUT_MS;
}

bool isCalibrationMode() {
    // Auto-clear calibration mode after 500ms timeout
    portENTER_CRITICAL(&cmdMux);
    uint32_t t = lastCalibrationTime;
    portEXIT_CRITICAL(&cmdMux);
    if (calibrationMode && (millis() - t) > VELOCITY_TIMEOUT_MS) {
        calibrationMode = false;
        stopAllMotors();
        // Also tear down the motor-cal state machine if it was driving the
        // motors — otherwise updateMotorCalibration() would resume them on
        // the next tick.
        if (isMotorCalibrationRunning()) {
            abortMotorCalibration();
        }
    }
    return calibrationMode;
}

uint32_t getTimeSinceLastCommand() {
    portENTER_CRITICAL(&cmdMux);
    uint32_t t = lastCommandTime;
    portEXIT_CRITICAL(&cmdMux);
    return millis() - t;
}

bool isHeadingHoldEnabled() {
    portENTER_CRITICAL(&headingHoldMux);
    bool en = headingHoldEnabled;
    portEXIT_CRITICAL(&headingHoldMux);
    return en;
}

float getHeadingHoldGain() {
    portENTER_CRITICAL(&headingHoldMux);
    float v = headingHoldGain;
    portEXIT_CRITICAL(&headingHoldMux);
    return v;
}

float getHeadingHoldDeadzone() {
    portENTER_CRITICAL(&headingHoldMux);
    float v = headingHoldDeadzone;
    portEXIT_CRITICAL(&headingHoldMux);
    return v;
}

float getHeadingHoldAlpha() {
    portENTER_CRITICAL(&headingHoldMux);
    float v = headingHoldAlpha;
    portEXIT_CRITICAL(&headingHoldMux);
    return v;
}

bool isNtpSynced() {
    return ntpSynced;
}

void autoLoadIMUCalibration() {
    if (!isIMUAvailable()) return;

    preferences.begin("imu_cal", true);
    bool valid = preferences.getBool("valid", false);
    if (valid) {
        adafruit_bno055_offsets_t offsets;
        size_t len = preferences.getBytes("offsets", &offsets, sizeof(offsets));
        if (len != sizeof(offsets)) {
            Serial.println("IMU calibration NVS load REJECTED: size mismatch");
            preferences.end();
            return;
        }
        // Sanity-check offset magnitudes (BNO055 offsets are typically < 500)
        const int16_t OFFSET_LIMIT = 2000;
        bool sane = true;
        int16_t* raw = (int16_t*)&offsets;
        for (size_t i = 0; i < sizeof(offsets) / sizeof(int16_t); i++) {
            if (raw[i] < -OFFSET_LIMIT || raw[i] > OFFSET_LIMIT) {
                sane = false;
                break;
            }
        }
        if (sane) {
            setIMUCalibrationOffsets(offsets);
            Serial.println("IMU calibration auto-loaded from NVS");
        } else {
            Serial.println("IMU calibration NVS load REJECTED: offset values out of range");
        }
    }
    preferences.end();
}

void autoLoadMotorCalibration() {
    preferences.begin("motor_cal", true);
    bool valid = preferences.getBool("valid", false);
    if (valid) {
        float gFwd[4], gRev[4];
        // Sanity range on loaded gains. A good calibration produces gains
        // close to 1.0. If a stored key is missing (getFloat returns the
        // default 1.0f — indistinguishable from a good value) or the NVS
        // page was corrupted mid-write, we could otherwise apply NaN/Inf
        // or wildly-skewed values and poison the PID until the next cal.
        const float MIN_GAIN = 0.5f;
        const float MAX_GAIN = 2.0f;
        bool allOk = true;
        for (int i = 0; i < 4 && allOk; i++) {
            char kf[8], kr[8];
            snprintf(kf, sizeof(kf), "gf_%d", i);
            snprintf(kr, sizeof(kr), "gr_%d", i);
            gFwd[i] = preferences.getFloat(kf, NAN);
            gRev[i] = preferences.getFloat(kr, NAN);
            if (isnan(gFwd[i]) || isinf(gFwd[i]) || gFwd[i] < MIN_GAIN || gFwd[i] > MAX_GAIN ||
                isnan(gRev[i]) || isinf(gRev[i]) || gRev[i] < MIN_GAIN || gRev[i] > MAX_GAIN) {
                allOk = false;
            }
        }
        if (allOk) {
            setMotorGains(gFwd, gRev);
            Serial.println("Motor calibration auto-loaded from NVS");
        } else {
            Serial.println("Motor calibration NVS load REJECTED: values out of range; keeping unity gains");
        }
    }
    preferences.end();
}
