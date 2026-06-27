#pragma once

// On-board teleop control page, served at GET /control.
//
// Self-contained HTML/CSS/JS (no external assets — works in AP mode).
// Stored in flash (PROGMEM) and served with request->send_P(), so the
// page never copies into the heap. It talks to the EXISTING /ws endpoint
// using the EXISTING {"type":"cmd",vx,vy,w} / {"type":"stop"} protocol —
// no firmware protocol changes.
//
// Axis convention (see firmware/esp32-omni/CLAUDE.md):
//   vx = forward (+X, m/s), vy = left (+Y, m/s), w = CCW (rad/s).
//
// Watchdog note: motors auto-stop VELOCITY_TIMEOUT_MS (500 ms) after the
// last cmd. The page therefore re-sends the current command on a 150 ms
// heartbeat while a control is held, and sends one `stop` on release.
//
// `static` gives the array internal linkage; the file is only included by
// websocket_server.cpp but this keeps it safe if that ever changes.
static const char CONTROL_HTML[] PROGMEM = R"rawliteral(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>Omni-2 Control</title>
<style>
:root{--bg:#16191d;--card:#23272e;--btn:#2f3742;--btnA:#4CAF50;--txt:#e6e9ee;--mut:#8b939e;--stop:#e23b3b}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
html,body{height:100%}
body{margin:0;font-family:system-ui,sans-serif;background:var(--bg);color:var(--txt);
  display:flex;flex-direction:column;align-items:center;padding:12px;gap:10px;touch-action:none}
h1{font-size:1.05rem;margin:0;font-weight:600}
.bar{display:flex;align-items:center;gap:8px;width:100%;max-width:420px;
  justify-content:space-between;flex:0 0 auto}
#dot{width:12px;height:12px;border-radius:50%;background:var(--stop);transition:background .2s}
#dot.ok{background:var(--btnA)}
.stage{display:flex;flex-direction:column;gap:10px;width:100%;max-width:420px}
.pad{display:grid;grid-template-columns:repeat(3,1fr);grid-template-rows:repeat(3,1fr);gap:8px}
.side{display:flex;flex-direction:column;gap:10px}
.rot{display:grid;grid-template-columns:repeat(2,1fr);grid-template-rows:1fr;gap:8px}
.sliders{display:flex;flex-direction:column;gap:8px}
.ctrl{background:var(--btn);border:none;border-radius:10px;color:var(--txt);
  font-size:1.4rem;height:72px;display:flex;align-items:center;justify-content:center;
  user-select:none;cursor:pointer;transition:background .07s}
.ctrl.on{background:var(--btnA)}
.estop{background:var(--stop);font-size:1rem;font-weight:700}
.row{display:flex;gap:8px;align-items:center}
.row label{font-size:.8rem;color:var(--mut);min-width:52px}
input[type=range]{flex:1;min-width:0;accent-color:var(--btnA)}
.val{font:12px ui-monospace,monospace;min-width:54px;text-align:right}
.tele{width:100%;max-width:420px;background:var(--card);border-radius:8px;padding:6px 10px;
  font:12px/1.4 ui-monospace,monospace;color:var(--mut);flex:0 0 auto}
.tele b{color:var(--txt)}
.hint{font-size:.72rem;color:var(--mut);max-width:420px;text-align:center}

/* Landscape (phone held sideways): gamepad layout — D-pad fills the
   height on the left, rotate + sliders on the right. Uses the full
   viewport, no scroll, big touch targets. */
@media (orientation:landscape){
  body{height:100dvh;padding:6px;gap:6px;overflow:hidden}
  h1{font-size:.95rem}
  .bar{max-width:none}
  .hint{display:none}
  .stage{flex-direction:row;max-width:none;flex:1 1 auto;min-height:0;align-items:stretch;gap:8px}
  .pad{flex:0 0 auto;aspect-ratio:1/1;height:100%}
  .side{flex:1 1 auto;min-width:0}
  .rot{flex:1 1 auto;min-height:0}
  .sliders{flex:0 0 auto}
  .ctrl{height:auto;min-height:0}
  .tele{max-width:none}
}
</style>
</head>
<body>
<div class="bar"><h1>Omni-2 Control</h1><span><span id="dot"></span> <span id="st">connecting</span></span></div>

<div class="stage">
<div class="pad">
  <button class="ctrl" data-vx="1"  data-vy="1"  id="bFL">&#8598;</button>
  <button class="ctrl" data-vx="1"               id="bF">&#8593;</button>
  <button class="ctrl" data-vx="1"  data-vy="-1" id="bFR">&#8599;</button>
  <button class="ctrl"             data-vy="1"   id="bL">&#8592;</button>
  <button class="ctrl estop"                     id="bSTOP">STOP</button>
  <button class="ctrl"             data-vy="-1"  id="bR">&#8594;</button>
  <button class="ctrl" data-vx="-1" data-vy="1"  id="bBL">&#8601;</button>
  <button class="ctrl" data-vx="-1"              id="bB">&#8595;</button>
  <button class="ctrl" data-vx="-1" data-vy="-1" id="bBR">&#8600;</button>
</div>

<div class="side">
<div class="rot">
  <button class="ctrl" data-w="1"  id="bCCW">&#8634; CCW</button>
  <button class="ctrl" data-w="-1" id="bCW">CW &#8635;</button>
</div>

<div class="sliders">
<div class="row"><label>Speed</label>
  <input type="range" id="lin" min="0.05" max="1.0" step="0.05" value="0.3">
  <span class="val" id="linV">0.30 m/s</span></div>
<div class="row"><label>Turn</label>
  <input type="range" id="ang" min="0.2" max="3.0" step="0.1" value="1.0">
  <span class="val" id="angV">1.0 rad/s</span></div>
</div>
</div>
</div>

<div class="tele" id="tele">cmd: 0,0,0</div>
<div class="hint">Keys: WASD / arrows move, Q/E rotate, Space = stop. Hold to drive.</div>

<script>
(function(){
"use strict";
var dot=document.getElementById("dot"),st=document.getElementById("st"),
    tele=document.getElementById("tele"),
    lin=document.getElementById("lin"),ang=document.getElementById("ang"),
    linV=document.getElementById("linV"),angV=document.getElementById("angV");
var ws=null, moving=false, active=new Set();

function fmtSliders(){linV.textContent=parseFloat(lin.value).toFixed(2)+" m/s";
  angV.textContent=parseFloat(ang.value).toFixed(1)+" rad/s";}
lin.oninput=ang.oninput=fmtSliders; fmtSliders();

function clamp(v){return v<-1?-1:v>1?1:v;}
function vector(){var sx=0,sy=0,sw=0;
  active.forEach(function(el){sx+=(+el.dataset.vx||0);sy+=(+el.dataset.vy||0);sw+=(+el.dataset.w||0);});
  return [clamp(sx),clamp(sy),clamp(sw)];}

function send(o){if(ws&&ws.readyState===1)ws.send(JSON.stringify(o));}

// Called on every input change AND on the heartbeat. Sends the live
// command while any control is held; sends a single stop on release.
function tick(){
  if(!ws||ws.readyState!==1)return;
  var v=vector(),L=parseFloat(lin.value),A=parseFloat(ang.value);
  if(v[0]||v[1]||v[2]){
    var vx=+(v[0]*L).toFixed(3),vy=+(v[1]*L).toFixed(3),w=+(v[2]*A).toFixed(3);
    send({type:"cmd",vx:vx,vy:vy,w:w});
    moving=true;
    tele.innerHTML="cmd: <b>"+vx+", "+vy+", "+w+"</b>";
  }else if(moving){
    send({type:"stop"}); moving=false; tele.innerHTML="cmd: 0,0,0";
  }
}
setInterval(tick,150);  // heartbeat — defeats the 500ms motor watchdog

function press(el){if(el&&!active.has(el)){active.add(el);el.classList.add("on");tick();}}
function release(el){if(el&&active.has(el)){active.delete(el);el.classList.remove("on");tick();}}
function estop(){active.forEach(function(e){e.classList.remove("on");});active.clear();
  send({type:"stop"});moving=false;tele.innerHTML="cmd: 0,0,0 (STOP)";}

// Pointer events cover mouse + touch with one API.
document.querySelectorAll(".ctrl").forEach(function(b){
  if(b.id==="bSTOP"){b.addEventListener("pointerdown",function(e){e.preventDefault();estop();});return;}
  b.addEventListener("pointerdown",function(e){e.preventDefault();press(b);});
  b.addEventListener("pointerup",function(e){e.preventDefault();release(b);});
  b.addEventListener("pointercancel",function(){release(b);});
  b.addEventListener("pointerleave",function(){release(b);});
});

// Keyboard maps onto the same on-screen buttons (shared highlight + state).
var KEYS={KeyW:"bF",ArrowUp:"bF",KeyS:"bB",ArrowDown:"bB",KeyA:"bL",ArrowLeft:"bL",
  KeyD:"bR",ArrowRight:"bR",KeyQ:"bCCW",KeyE:"bCW"};
addEventListener("keydown",function(e){
  if(e.code==="Space"){e.preventDefault();estop();return;}
  var id=KEYS[e.code]; if(!id||e.repeat)return; e.preventDefault();
  press(document.getElementById(id));});
addEventListener("keyup",function(e){
  var id=KEYS[e.code]; if(!id)return; e.preventDefault();
  release(document.getElementById(id));});

// Safety: halt if the page loses focus / is hidden / is being unloaded.
function panic(){if(active.size||moving)estop();}
addEventListener("blur",panic);
addEventListener("pagehide",panic);
document.addEventListener("visibilitychange",function(){if(document.hidden)panic();});

function setStatus(ok,txt){dot.classList.toggle("ok",ok);st.textContent=txt;}

function connect(){
  ws=new WebSocket("ws://"+location.host+"/ws");
  ws.onopen=function(){setStatus(true,"connected");send({type:"get_info"});};
  ws.onclose=function(){setStatus(false,"reconnecting");moving=false;setTimeout(connect,1000);};
  ws.onerror=function(){ws.close();};
  ws.onmessage=function(ev){
    var m; try{m=JSON.parse(ev.data);}catch(_){return;}
    if(m.type==="sensors"&&m.pose){
      var h=m.heap?(" | heap "+(m.heap.free/1024|0)+"k"):"";
      tele.title="pose "+m.pose.x.toFixed(2)+","+m.pose.y.toFixed(2)+
        " th"+(m.pose.th*57.3).toFixed(0)+"°"+h;
    }
  };
}
connect();
})();
</script>
</body>
</html>)rawliteral";
