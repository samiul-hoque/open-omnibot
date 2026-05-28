# Localization Methods for Mecanum Wheel Robot

This document describes the three localization tiers implemented for the Omni-2 robot platform. Each tier builds upon the previous one, adding more sensors and sophistication to improve pose estimation accuracy.

## Table of Contents

1. [Robot Kinematics Background](#robot-kinematics-background)
2. [Tier 1: Dead Reckoning Odometry](#tier-1-dead-reckoning-odometry)
3. [Tier 2: Encoder + IMU Fusion](#tier-2-encoder--imu-fusion)
4. [Tier 3: Extended Kalman Filter (EKF)](#tier-3-extended-kalman-filter-ekf)
5. [Comparison and Trade-offs](#comparison-and-trade-offs)

---

## Robot Kinematics Background

### Mecanum Wheel Configuration

The Omni-2 robot uses four mecanum wheels arranged in a standard configuration. Mecanum wheels have rollers mounted at 45° angles, which allows the robot to move in any direction without changing its orientation—unlike regular wheels that can only go forward/backward.

```
        Front
    L1 ───────── R1
     │           │
     │     ↑     │
     │    +x     │
     │           │
    L2 ───────── R2
        Rear
```

Each wheel has rollers mounted at 45° angles, enabling omnidirectional motion.

### Coordinate System

- **+x**: Forward (front of robot)
- **+y**: Left (strafe direction)
- **+θ**: Counter-clockwise rotation (CCW)

### Physical Parameters

| Parameter | Symbol | Value | Description |
|-----------|--------|-------|-------------|
| Wheel radius | r | 0.04 m | 80mm diameter wheels |
| Half track width | lₓ | 0.135 m | Distance from center to wheel (x-axis) |
| Half wheelbase | lᵧ | 0.135 m | Distance from center to wheel (y-axis) |
| Encoder resolution | N | 1092 counts/rev | Encoder counts per wheel revolution |

### Mecanum Forward Kinematics

**What we're solving:** Given how much each wheel rotated, determine how the robot moved.

The relationship between wheel angular velocities (ω₁, ω₂, ω₃, ω₄) and body velocities (vₓ, vᵧ, ω) is given by the forward kinematics matrix:

$$
\begin{bmatrix} v_x \\ v_y \\ \omega \end{bmatrix} = \frac{r}{4} \begin{bmatrix} 1 & 1 & 1 & 1 \\ -1 & 1 & -1 & 1 \\ \frac{-1}{l_x + l_y} & \frac{1}{l_x + l_y} & \frac{1}{l_x + l_y} & \frac{-1}{l_x + l_y} \end{bmatrix} \begin{bmatrix} \omega_{L1} \\ \omega_{R1} \\ \omega_{R2} \\ \omega_{L2} \end{bmatrix}
$$

Expanding this matrix multiplication gives us three equations:

**Forward/Backward velocity (vₓ):**
$$v_x = \frac{r}{4}(\omega_{L1} + \omega_{R1} + \omega_{R2} + \omega_{L2})$$

*Intuition: All four wheels contribute equally. When all wheels spin forward, their contributions add up to forward motion.*

**Lateral velocity (vᵧ) - Strafing:**
$$v_y = \frac{r}{4}(-\omega_{L1} + \omega_{R1} - \omega_{R2} + \omega_{L2})$$

*Intuition: Notice the alternating signs! This is the magic of mecanum wheels. The 45° rollers cause left wheels to push in one direction and right wheels in another, enabling sideways motion.*

**Angular velocity (ω) - Rotation:**
$$\omega = \frac{r}{4(l_x + l_y)}(-\omega_{L1} + \omega_{R1} + \omega_{R2} - \omega_{L2})$$

*Intuition: Left wheels spinning backward + right wheels spinning forward = robot rotates counter-clockwise. The (lₓ + lᵧ) term accounts for the robot's size—larger robots rotate slower for the same wheel speeds.*

### Understanding the Sign Patterns

The key insight is understanding why each formula has its specific sign pattern. Let's verify with examples:

**Example 1: Forward Motion (all wheels forward)**
- ωL1 = ωR1 = ωR2 = ωL2 = +ω

| Formula | Calculation | Result |
|---------|-------------|--------|
| vₓ | (+) + (+) + (+) + (+) = 4ω | Forward ✓ |
| vᵧ | (-) + (+) + (-) + (+) = 0 | No strafe ✓ |
| ω | (-) + (+) + (+) + (-) = 0 | No rotation ✓ |

**Example 2: Right Strafe (L wheels forward, R wheels backward)**
- ωL1 = +ω, ωR1 = -ω, ωR2 = +ω, ωL2 = -ω

| Formula | Calculation | Result |
|---------|-------------|--------|
| vₓ | (+) + (-) + (+) + (-) = 0 | No forward ✓ |
| vᵧ | -(+) + (-) + -(+) + (-) = -4ω | Right strafe ✓ |
| ω | -(+) + (-) + (+) + -(-) = 0 | No rotation ✓ |

**Example 3: Counter-clockwise Rotation (L wheels back, R wheels forward)**
- ωL1 = -ω, ωR1 = +ω, ωR2 = +ω, ωL2 = -ω

| Formula | Calculation | Result |
|---------|-------------|--------|
| vₓ | (-) + (+) + (+) + (-) = 0 | No forward ✓ |
| vᵧ | -(-) + (+) + -(+) + (-) = 0 | No strafe ✓ |
| ω | -(-) + (+) + (+) - (-) = 4ω | CCW rotation ✓ |

---

## Tier 1: Dead Reckoning Odometry

**File:** `src/localization/odometry.js`

### Overview

Dead reckoning uses only wheel encoder measurements to estimate the robot's pose. It integrates wheel displacements over time to track position and orientation.

*Think of it like counting your steps to know how far you've walked—simple and requires no external references, but errors accumulate over time.*

### Algorithm

#### Step 1: Compute Encoder Deltas

For each wheel, calculate the change in encoder counts since the last update:

$$\Delta_i = (E_i^{(t)} - E_i^{(t-1)}) \cdot s_i$$

Where:
- $E_i^{(t)}$ = current encoder count for wheel i
- $E_i^{(t-1)}$ = previous encoder count
- $s_i$ = sign correction (+1 or -1) for reversed motors

*This gives us how many "ticks" each wheel moved since last time.*

#### Step 2: Convert to Wheel Displacements

Convert encoder counts to linear displacement (meters):

$$d_i = \Delta_i \cdot \frac{2\pi r}{N}$$

Where $\frac{2\pi r}{N}$ is the meters per encoder count.

*With our parameters: 2π × 0.04m / 1092 ≈ 0.00023 m per tick, so each encoder tick represents about 0.23mm of wheel travel.*

#### Step 3: Compute Wheel Angular Displacements

$$\omega_i = \frac{d_i}{r}$$

*This converts linear distance back to angular displacement (radians), which is what the kinematics equations expect.*

#### Step 4: Apply Forward Kinematics

Compute body-frame displacements using the mecanum equations:

$$dx_{robot} = \frac{r}{4}(\omega_{L1} + \omega_{R1} + \omega_{R2} + \omega_{L2})$$

$$dy_{robot} = \frac{r}{4}(-\omega_{L1} + \omega_{R1} - \omega_{R2} + \omega_{L2})$$

$$d\theta = \frac{r}{4(l_x + l_y)}(-\omega_{L1} + \omega_{R1} + \omega_{R2} - \omega_{L2})$$

*These give us how far the robot moved in its own reference frame (forward, sideways, and rotation).*

#### Step 5: Rotation Compensation

During pure rotation, wheel slip causes spurious position changes. We detect and suppress this:

```javascript
if (|dθ| > 0.002) {
    ratio = √(dx² + dy²) / |dθ|
    if (ratio < 0.15) {
        // Translation is tiny compared to rotation
        // Likely wheel slip noise, not real movement
        dx_robot *= 0.2
        dy_robot *= 0.2
    }
}
```

*When spinning in place, the wheels slip slightly and report small phantom translations. This heuristic detects "mostly rotation" and reduces the fake translation to 20%.*

#### Step 6: Transform to World Frame

The robot calculates movement in its own reference frame, but we need absolute world coordinates. We use a rotation matrix with the midpoint angle for better accuracy:

$$\theta_{mid} = \theta + \frac{d\theta}{2}$$

$$\begin{bmatrix} dx_{world} \\ dy_{world} \end{bmatrix} = \begin{bmatrix} \cos(\theta_{mid}) & -\sin(\theta_{mid}) \\ \sin(\theta_{mid}) & \cos(\theta_{mid}) \end{bmatrix} \begin{bmatrix} dx_{robot} \\ dy_{robot} \end{bmatrix}$$

Expanded:
$$dx_{world} = dx_{robot} \cos(\theta_{mid}) - dy_{robot} \sin(\theta_{mid})$$
$$dy_{world} = dx_{robot} \sin(\theta_{mid}) + dy_{robot} \cos(\theta_{mid})$$

*If the robot is pointing 90° left, then its "forward" becomes world's "left" and its "left" becomes world's "backward". The rotation matrix handles this transformation for any angle.*

*Using the midpoint angle θ + dθ/2 instead of the starting angle improves accuracy when the robot rotates during the update interval.*

#### Step 7: Update Pose

$$x \leftarrow x + dx_{world}$$
$$y \leftarrow y + dy_{world}$$
$$\theta \leftarrow \text{normalize}(\theta + d\theta)$$

Where normalize() keeps the angle in $[-\pi, \pi]$:
```javascript
while (angle > π) angle -= 2π
while (angle < -π) angle += 2π
```

### Error Characteristics

| Error Source | Effect | Accumulation |
|--------------|--------|--------------|
| Wheel slip | Position drift | Unbounded |
| Encoder quantization | Small position noise | Bounded |
| Wheel diameter variation | Systematic bias | Linear with distance |
| Heading error | Growing position error | Quadratic with distance |

**Critical insight:** Heading errors are particularly damaging because they cause the robot to integrate position in the wrong direction. A 5° heading error after 10m of travel causes ~0.87m of lateral position error.

### Advantages

- Simple implementation
- Low computational cost
- No external sensors required
- Works in any environment

### Disadvantages

- Unbounded drift over time
- Sensitive to wheel slip
- Heading errors compound into position errors
- No absolute reference

---

## Tier 2: Encoder + IMU Fusion

**File:** `src/localization/fusionBasic.js`

### Overview

This tier adds an Inertial Measurement Unit (IMU) to provide absolute heading information. A complementary filter blends the encoder-based heading with the IMU yaw measurement, significantly reducing heading drift.

*The IMU (specifically the BNO055) contains a gyroscope, accelerometer, and magnetometer. It fuses these internally to provide a stable absolute heading that doesn't drift like encoder-based heading.*

### Sensor Fusion Strategy

The complementary filter exploits the complementary strengths of each sensor:

| Sensor | Strength | Weakness |
|--------|----------|----------|
| Encoders | Good short-term accuracy, responsive | Drifts over time |
| IMU | Stable absolute heading | Subject to magnetic interference, noise |

*The key insight: encoders are good for quick changes but drift; IMU is good for long-term stability but noisy. By combining them, we get the best of both.*

### Algorithm

#### Steps 1-5: Same as Tier 1

Compute encoder-based odometry exactly as in Tier 1, obtaining $dx_{robot}$, $dy_{robot}$, and $d\theta_{odom}$.

#### Step 6: IMU Initialization

On the first IMU reading, we align the IMU frame with our odometry frame by computing an offset:

$$\text{offset}_{IMU} = \psi_{IMU} - \theta_{odom}$$

Where $\psi_{IMU}$ is the raw IMU yaw in radians.

*This offset accounts for the fact that the IMU's "zero" might not match our odometry's "zero". We subtract this offset from all future IMU readings.*

#### Step 7: Complementary Filter for Heading

The complementary filter blends two heading estimates:

**Get both heading estimates:**
$$\theta_{IMU} = \text{normalize}(\psi_{IMU} - \text{offset}_{IMU})$$
$$\theta_{odom} = \text{normalize}(\theta_{prev} + d\theta_{odom})$$

**Compute the difference (handling angle wrapping):**
$$\Delta = \text{normalize}(\theta_{IMU} - \theta_{odom})$$

**Blend them using weight α:**
$$\theta_{new} = \text{normalize}(\theta_{odom} + \alpha \cdot \Delta)$$

With α = 0.98 (our default), this is equivalent to:
$$\theta_{new} = 0.02 \cdot \theta_{odom} + 0.98 \cdot \theta_{IMU}$$

### Complementary Filter Theory

The complementary filter can be understood in the frequency domain:

$$\theta_{fused} = G_{HP}(s) \cdot \theta_{encoders} + G_{LP}(s) \cdot \theta_{IMU}$$

Where:
- $G_{HP}(s)$ = high-pass filter (passes quick changes from encoders)
- $G_{LP}(s)$ = low-pass filter (passes stable baseline from IMU)
- $G_{HP}(s) + G_{LP}(s) = 1$ (complementary property)

In our discrete implementation with weight α = 0.98:
- **Low frequencies** (slow drift): 98% trust in IMU
- **High frequencies** (fast motion): 2% from encoder changes

*Intuitively: The IMU provides the stable "anchor" for heading, while encoder changes add the quick responsiveness. The high α value means we strongly trust the IMU's absolute reference.*

### Position Calculation

Position still comes from encoder odometry, but transformed using the **fused heading**:

$$\theta_{mid} = \frac{\theta_{prev} + \theta_{new}}{2}$$

$$dx_{world} = dx_{robot} \cos(\theta_{mid}) - dy_{robot} \sin(\theta_{mid})$$
$$dy_{world} = dx_{robot} \sin(\theta_{mid}) + dy_{robot} \cos(\theta_{mid})$$

*Important: Only heading is improved by the IMU. Position (X, Y) still comes from encoder integration and will still drift—but with correct heading, the drift is much slower and more predictable.*

### IMU Weight Selection Guide

| Weight (α) | Behavior | Use Case |
|------------|----------|----------|
| 0.99 | Almost pure IMU heading | Very stable environment, trust IMU completely |
| 0.98 | Default - strong IMU trust | Normal operation |
| 0.90 | Moderate blend | Some magnetic interference |
| 0.50 | Equal trust | Testing/comparison |
| 0.10 | Mostly encoder-based | Severe magnetic interference |

### Error Characteristics

| Error Source | Effect | Compared to Tier 1 |
|--------------|--------|-------------------|
| Heading drift | Eliminated by IMU | **Much better** |
| Position drift | Still accumulates | Similar |
| IMU magnetic interference | Temporary heading error | New issue |

### Advantages

- Eliminates heading drift
- Simple to implement and tune
- Low computational cost
- Single tunable parameter (α)

### Disadvantages

- Position still drifts (no absolute position reference)
- Susceptible to magnetic interference (motors, metal structures)
- Requires IMU calibration
- IMU must be properly mounted (vibration isolation)

---

## Tier 3: Extended Kalman Filter (EKF)

**File:** `src/localization/fusionEKF.js`

### Overview

The Extended Kalman Filter provides optimal sensor fusion by explicitly modeling uncertainty. It combines:
- **Prediction**: Encoder odometry with process noise
- **Update (Heading)**: IMU yaw measurements
- **Update (Position)**: UWB absolute position

*The EKF is "optimal" in that it minimizes the expected estimation error given the noise characteristics of each sensor. It automatically balances trust between prediction and measurement based on their respective uncertainties.*

### State Vector

The robot's pose is represented as a state vector:

$$\mathbf{x} = \begin{bmatrix} x \\ y \\ \theta \end{bmatrix}$$

Where:
- $x$ = position in world X coordinate (meters)
- $y$ = position in world Y coordinate (meters)
- $\theta$ = heading angle (radians)

### Covariance Matrix

The uncertainty in our state estimate is captured by the covariance matrix:

$$\mathbf{P} = \begin{bmatrix} \sigma_x^2 & \sigma_{xy} & \sigma_{x\theta} \\ \sigma_{xy} & \sigma_y^2 & \sigma_{y\theta} \\ \sigma_{x\theta} & \sigma_{y\theta} & \sigma_\theta^2 \end{bmatrix}$$

*In our simplified implementation, we use a diagonal matrix (off-diagonal terms = 0), treating x, y, and θ as independent:*

$$\mathbf{P} = \begin{bmatrix} P_{xx} & 0 & 0 \\ 0 & P_{yy} & 0 \\ 0 & 0 & P_{\theta\theta} \end{bmatrix}$$

*The diagonal elements represent our uncertainty (variance) in each state variable. Larger values = less confident.*

### The Kalman Filter Cycle

The EKF alternates between two steps:

```
┌─────────────────────────────────────────────────────┐
│                    PREDICT                          │
│  Use motion model (encoders) to predict new state   │
│  Uncertainty INCREASES (we're less sure)            │
└─────────────────────┬───────────────────────────────┘
                      │
                      ▼
┌─────────────────────────────────────────────────────┐
│                    UPDATE                           │
│  Use sensor measurement to correct prediction       │
│  Uncertainty DECREASES (measurement adds info)      │
└─────────────────────────────────────────────────────┘
```

### Prediction Step

When encoder data arrives, we predict the new state based on odometry:

#### State Prediction

$$\hat{\mathbf{x}} = f(\mathbf{x}, \mathbf{u}) = \begin{bmatrix} x + dx \\ y + dy \\ \text{normalize}(\theta + d\theta) \end{bmatrix}$$

Where $(dx, dy, d\theta)$ come from the encoder odometry equations (same as Tier 1).

*This is our "best guess" of where the robot is now, based on where it was and how we think it moved.*

#### Covariance Prediction (Simplified)

For our diagonal implementation:

$$P_{xx} \leftarrow P_{xx} + Q_x$$
$$P_{yy} \leftarrow P_{yy} + Q_y$$
$$P_{\theta\theta} \leftarrow P_{\theta\theta} + Q_\theta$$

*Each prediction step INCREASES our uncertainty by adding process noise Q. This reflects the fact that our motion model isn't perfect—wheel slip, encoder noise, etc. add uncertainty.*

**Process Noise Parameters:**

| Parameter | Default Value | Physical Meaning |
|-----------|---------------|------------------|
| $Q_x$ | 0.01 m² | Uncertainty added per step in X |
| $Q_y$ | 0.01 m² | Uncertainty added per step in Y |
| $Q_\theta$ | 0.005 rad² | Uncertainty added per step in heading |

#### Full Covariance Prediction (for reference)

The complete EKF prediction would use the Jacobian of the motion model:

$$\mathbf{P} \leftarrow \mathbf{F} \mathbf{P} \mathbf{F}^T + \mathbf{Q}$$

Where $\mathbf{F} = \frac{\partial f}{\partial \mathbf{x}}$ is the Jacobian. Our simplified diagonal form is an approximation that works well in practice.

### IMU Heading Update

When IMU data arrives, we update our heading estimate:

#### Innovation (Measurement Residual)

$$\tilde{y}_\theta = \text{normalize}(\theta_{IMU} - \hat{\theta})$$

*The innovation is "what we measured minus what we predicted." It tells us how wrong our prediction was.*

#### Innovation Covariance

$$S_\theta = P_{\theta\theta} + R_\theta$$

*This combines our prediction uncertainty with the sensor's measurement uncertainty.*

Where $R_\theta$ is the IMU measurement noise variance (default: 0.01 rad²).

#### Kalman Gain

$$K_\theta = \frac{P_{\theta\theta}}{S_\theta} = \frac{P_{\theta\theta}}{P_{\theta\theta} + R_\theta}$$

*The Kalman gain determines how much to trust the measurement vs. the prediction. This formula is derived from minimizing the expected squared error.*

**Intuitive interpretation:**
- If $P_{\theta\theta} \gg R_\theta$ (we're very uncertain, sensor is precise): $K \to 1$ (trust sensor)
- If $P_{\theta\theta} \ll R_\theta$ (we're confident, sensor is noisy): $K \to 0$ (trust prediction)
- If $P_{\theta\theta} = R_\theta$ (equal uncertainty): $K = 0.5$ (average them)

#### State Update

$$\theta \leftarrow \text{normalize}(\hat{\theta} + K_\theta \cdot \tilde{y}_\theta)$$

*We correct our prediction by adding a fraction (K) of the innovation.*

#### Covariance Update

$$P_{\theta\theta} \leftarrow (1 - K_\theta) \cdot P_{\theta\theta}$$

*After incorporating the measurement, our uncertainty DECREASES. The more we trusted the measurement (higher K), the more our uncertainty drops.*

### UWB Position Update

When UWB position data arrives, we update our position estimate:

#### Measurement Model

$$\mathbf{z} = \begin{bmatrix} z_x \\ z_y \end{bmatrix} = \begin{bmatrix} x \\ y \end{bmatrix} + \mathbf{v}$$

Where $\mathbf{v}$ is measurement noise with covariance $\mathbf{R}_{UWB}$.

*The UWB system directly measures X and Y position (plus noise).*

#### Innovation

$$\tilde{y}_x = z_x - \hat{x}$$
$$\tilde{y}_y = z_y - \hat{y}$$

#### Innovation Covariance

$$S_x = P_{xx} + R_x$$
$$S_y = P_{yy} + R_y$$

Where $R_x, R_y$ are UWB measurement noise variances (default: 0.15 m² each, representing ~38cm standard deviation).

#### Kalman Gain

$$K_x = \frac{P_{xx}}{S_x} = \frac{P_{xx}}{P_{xx} + R_x}$$
$$K_y = \frac{P_{yy}}{S_y} = \frac{P_{yy}}{P_{yy} + R_y}$$

#### State Update

$$x \leftarrow \hat{x} + K_x \cdot \tilde{y}_x$$
$$y \leftarrow \hat{y} + K_y \cdot \tilde{y}_y$$

#### Covariance Update

$$P_{xx} \leftarrow (1 - K_x) \cdot P_{xx}$$
$$P_{yy} \leftarrow (1 - K_y) \cdot P_{yy}$$

### Why UWB is Critical

Ultra-Wideband (UWB) positioning provides **absolute position** from fixed anchors. Unlike encoders that drift forever, UWB errors don't accumulate:

| Without UWB | With UWB |
|-------------|----------|
| Position error grows unbounded | Position error stays bounded |
| Error ∝ distance traveled | Error ≈ constant (UWB noise) |
| Cannot recover from drift | Continuously corrects drift |

*The EKF allows encoder-based navigation between UWB updates, with UWB periodically "snapping" the position back to reality.*

### Sensor Fusion Flow Diagram

```
┌─────────────┐
│  Encoders   │──────┐
│  (50 Hz)    │      │
└─────────────┘      │
                     ▼
              ┌──────────────┐
              │   PREDICT    │
              │  x̂ = f(x,u)  │
              │  P = P + Q   │
              └──────┬───────┘
                     │
       ┌─────────────┼─────────────┐
       ▼             ▼             ▼
┌──────────┐  ┌──────────┐  ┌──────────┐
│   IMU    │  │   UWB    │  │  Future  │
│ (50 Hz)  │  │ (10 Hz)  │  │ Sensors  │
└────┬─────┘  └────┬─────┘  └────┬─────┘
     │             │             │
     ▼             ▼             ▼
  UPDATE θ     UPDATE x,y    UPDATE ...
  K = P/(P+R)  K = P/(P+R)
  x += K·ỹ    x += K·ỹ
  P *= (1-K)  P *= (1-K)
     │             │             │
     └─────────────┴─────────────┘
                   │
                   ▼
            ┌─────────────┐
            │ Final Pose  │
            │   (x,y,θ)   │
            │  with P     │
            └─────────────┘
```

### Tuning Guidelines

| If you observe... | Likely cause | Adjust... |
|-------------------|--------------|-----------|
| Position jumps when UWB updates | UWB noise too low (over-trusted) | Increase $R_x$, $R_y$ |
| Slow response to UWB | UWB noise too high (under-trusted) | Decrease $R_x$, $R_y$ |
| Position drifts between UWB | Process noise too low | Increase $Q_x$, $Q_y$ |
| Jittery position | Process noise too high | Decrease $Q_x$, $Q_y$ |
| Noisy heading | IMU noise too low | Increase $R_\theta$ |
| Sluggish heading response | IMU noise too high | Decrease $R_\theta$ |

### Advantages

- Optimal sensor fusion (minimum variance estimate)
- Explicit uncertainty tracking (know how confident you are)
- Bounded position error (with UWB)
- Graceful sensor degradation (works even if sensors drop out temporarily)
- Extensible (easy to add more sensors)

### Disadvantages

- Higher computational cost
- Requires UWB infrastructure (anchors)
- More parameters to tune
- More complex debugging
- Linearization can cause issues with large errors (Extended KF limitation)

---

## Comparison and Trade-offs

### Feature Comparison

| Feature | Tier 1 | Tier 2 | Tier 3 |
|---------|--------|--------|--------|
| Position accuracy | Poor (drifts) | Poor (drifts) | Good (bounded) |
| Heading accuracy | Poor (drifts) | Good (stable) | Good (stable) |
| Computational cost | Low | Low | Medium |
| Sensor requirements | Encoders only | Encoders + IMU | Encoders + IMU + UWB |
| Infrastructure | None | None | UWB anchors |
| Uncertainty estimate | No | No | Yes (covariance P) |

### When to Use Each Tier

| Scenario | Recommended Tier | Reasoning |
|----------|------------------|-----------|
| Short-distance demos | Tier 1 | Drift hasn't accumulated yet |
| Outdoor/large area, no UWB | Tier 2 | IMU prevents heading disaster |
| Precision indoor positioning | Tier 3 | UWB bounds position error |
| No IMU available | Tier 1 | Only option |
| Thesis comparison study | All three | Show progressive improvement |

### Expected Accuracy

| Tier | Position Error (1m travel) | Position Error (10m travel) | Heading Error |
|------|---------------------------|----------------------------|---------------|
| 1 | ~5-10 cm | ~50-100 cm | ~5-10° drift |
| 2 | ~5-10 cm | ~50-100 cm | ~1-2° (stable) |
| 3 | ~5-15 cm | ~5-15 cm (bounded!) | ~1-2° (stable) |

*Note: Tier 3 position error doesn't grow with distance because UWB continuously corrects drift.*

### Error Growth Visualization

```
Position Error
     ^
     │
     │                              Tier 1 (encoders)
     │                           ╱
 1m  │                        ╱
     │                     ╱
     │                  ╱         Tier 2 (enc+IMU)
     │               ╱         ╱  (slower drift due to
     │            ╱         ╱      correct heading)
     │         ╱         ╱
     │      ╱         ╱
     │   ╱         ╱
15cm │╱─────────────────────────── Tier 3 (EKF+UWB)
     │                              (bounded!)
     └────────────────────────────> Distance traveled
```

---

## Implementation Notes

### Encoder Mapping

The firmware sends encoder data as `[L1, R1, L2, R2]`, but our kinematics equations expect `[L1, R1, R2, L2]`. We apply a mapping to swap the rear wheel indices:

```javascript
const mapping = [0, 1, 3, 2];  // Swap indices 2 and 3
const encoders = [raw[mapping[0]], raw[mapping[1]], raw[mapping[2]], raw[mapping[3]]];
// Result: [L1, R1, R2, L2]
```

### Angle Normalization

All angles are normalized to $[-\pi, \pi]$ to handle wraparound correctly:

```javascript
function normalizeAngle(angle) {
    while (angle > Math.PI) angle -= 2 * Math.PI;
    while (angle < -Math.PI) angle += 2 * Math.PI;
    return angle;
}
```

*This is essential for the complementary filter and EKF to work correctly when the robot crosses the ±180° boundary.*

### Rotation Compensation

Pure rotation causes wheel slip that appears as small phantom translations. We detect and suppress this:

```javascript
const rotationMagnitude = Math.abs(dtheta);
const translationMagnitude = Math.sqrt(dx_robot² + dy_robot²);

if (rotationMagnitude > 0.002) {  // Robot is rotating
    const ratio = translationMagnitude / rotationMagnitude;
    if (ratio < 0.15) {  // Translation is tiny compared to rotation
        // This is likely slip noise, not real translation
        dx_robot *= 0.2;
        dy_robot *= 0.2;
    }
}
```

---

## References

1. Siegwart, R., Nourbakhsh, I. R., & Scaramuzza, D. (2011). *Introduction to Autonomous Mobile Robots* (2nd ed.). MIT Press.

2. Thrun, S., Burgard, W., & Fox, D. (2005). *Probabilistic Robotics*. MIT Press.

3. Taheri, H., Qiao, B., & Ghaeminezhad, N. (2015). "Kinematic model of a four mecanum wheeled mobile robot." *International Journal of Computer Applications*, 113(3), 30-37.

4. Kalman, R. E. (1960). "A New Approach to Linear Filtering and Prediction Problems." *Journal of Basic Engineering*, 82(1), 35-45.

5. Welch, G., & Bishop, G. (2006). "An Introduction to the Kalman Filter." *UNC Chapel Hill TR 95-041*.
