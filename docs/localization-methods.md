# Localization Methods for Mecanum Wheel Robot

This document provides a formal mathematical treatment of the three localization tiers implemented for the Omni-2 robot platform.

## Table of Contents

1. [Robot Kinematics](#robot-kinematics)
2. [Tier 1: Dead Reckoning Odometry](#tier-1-dead-reckoning-odometry)
3. [Tier 2: Encoder + IMU Fusion](#tier-2-encoder--imu-fusion)
4. [Tier 3: Extended Kalman Filter (EKF)](#tier-3-extended-kalman-filter-ekf)
5. [Comparison](#comparison)

---

## Robot Kinematics

### Configuration

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

### Coordinate System

- **+x**: Forward
- **+y**: Left
- **+θ**: Counter-clockwise rotation

### Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Wheel radius | r | 0.04 m |
| Half track width | lₓ | 0.135 m |
| Half wheelbase | lᵧ | 0.135 m |
| Encoder resolution | N | 1092 counts/rev |

### Forward Kinematics

$$
\begin{bmatrix} v_x \\ v_y \\ \omega \end{bmatrix} = \frac{r}{4} \begin{bmatrix} 1 & 1 & 1 & 1 \\ -1 & 1 & -1 & 1 \\ \frac{-1}{l_x + l_y} & \frac{1}{l_x + l_y} & \frac{1}{l_x + l_y} & \frac{-1}{l_x + l_y} \end{bmatrix} \begin{bmatrix} \omega_{L1} \\ \omega_{R1} \\ \omega_{R2} \\ \omega_{L2} \end{bmatrix}
$$

Scalar form:

$$v_x = \frac{r}{4}(\omega_{L1} + \omega_{R1} + \omega_{R2} + \omega_{L2})$$

$$v_y = \frac{r}{4}(-\omega_{L1} + \omega_{R1} - \omega_{R2} + \omega_{L2})$$

$$\omega = \frac{r}{4(l_x + l_y)}(-\omega_{L1} + \omega_{R1} + \omega_{R2} - \omega_{L2})$$

---

## Tier 1: Dead Reckoning Odometry

**File:** `src/localization/odometry.js`

### Algorithm

#### Step 1: Encoder Deltas

$$\Delta_i = (E_i^{(t)} - E_i^{(t-1)}) \cdot s_i$$

Where:
- $E_i^{(t)}$ = current encoder count
- $s_i$ = sign correction

#### Step 2: Wheel Displacements

$$d_i = \Delta_i \cdot \frac{2\pi r}{N}$$

#### Step 3: Angular Displacements

$$\omega_i = \frac{d_i}{r}$$

#### Step 4: Body-Frame Kinematics

$$dx_{robot} = \frac{r}{4}(\omega_{L1} + \omega_{R1} + \omega_{R2} + \omega_{L2})$$

$$dy_{robot} = \frac{r}{4}(-\omega_{L1} + \omega_{R1} - \omega_{R2} + \omega_{L2})$$

$$d\theta = \frac{r}{4(l_x + l_y)}(-\omega_{L1} + \omega_{R1} + \omega_{R2} - \omega_{L2})$$

#### Step 5: Rotation Compensation

$$\text{if } |d\theta| > 0.002 \text{ and } \frac{\sqrt{dx^2 + dy^2}}{|d\theta|} < 0.15:$$
$$dx_{robot} \leftarrow 0.2 \cdot dx_{robot}$$
$$dy_{robot} \leftarrow 0.2 \cdot dy_{robot}$$

#### Step 6: World Frame Transformation

$$\theta_{mid} = \theta + \frac{d\theta}{2}$$

$$\begin{bmatrix} dx_{world} \\ dy_{world} \end{bmatrix} = \begin{bmatrix} \cos(\theta_{mid}) & -\sin(\theta_{mid}) \\ \sin(\theta_{mid}) & \cos(\theta_{mid}) \end{bmatrix} \begin{bmatrix} dx_{robot} \\ dy_{robot} \end{bmatrix}$$

#### Step 7: State Update

$$x \leftarrow x + dx_{world}$$
$$y \leftarrow y + dy_{world}$$
$$\theta \leftarrow \text{normalize}(\theta + d\theta)$$

### Error Characteristics

| Error Source | Accumulation |
|--------------|--------------|
| Wheel slip | Unbounded |
| Encoder quantization | Bounded |
| Wheel diameter variation | Linear |
| Heading error | Quadratic |

---

## Tier 2: Encoder + IMU Fusion

**File:** `src/localization/fusionBasic.js`

### Complementary Filter

#### IMU Initialization

$$\text{offset}_{IMU} = \psi_{IMU} - \theta_{odom}$$

#### Heading Estimates

$$\theta_{IMU} = \text{normalize}(\psi_{IMU} - \text{offset}_{IMU})$$

$$\theta_{odom} = \text{normalize}(\theta_{prev} + d\theta_{odom})$$

#### Filter Equation

$$\Delta = \text{normalize}(\theta_{IMU} - \theta_{odom})$$

$$\theta_{new} = \text{normalize}(\theta_{odom} + \alpha \cdot \Delta)$$

Equivalent form:

$$\theta_{fused} = (1 - \alpha) \cdot \theta_{encoders} + \alpha \cdot \theta_{IMU}$$

Default: $\alpha = 0.98$

### Frequency Domain Interpretation

$$\theta_{fused} = G_{HP}(s) \cdot \theta_{encoders} + G_{LP}(s) \cdot \theta_{IMU}$$

Where $G_{HP}(s) + G_{LP}(s) = 1$.

### Position Update

$$\theta_{mid} = \frac{\theta_{prev} + \theta_{new}}{2}$$

$$dx_{world} = dx_{robot} \cos(\theta_{mid}) - dy_{robot} \sin(\theta_{mid})$$

$$dy_{world} = dx_{robot} \sin(\theta_{mid}) + dy_{robot} \cos(\theta_{mid})$$

---

## Tier 3: Extended Kalman Filter (EKF)

**File:** `src/localization/fusionEKF.js`

### State Vector

$$\mathbf{x} = \begin{bmatrix} x \\ y \\ \theta \end{bmatrix}$$

### Covariance Matrix

$$\mathbf{P} = \begin{bmatrix} \sigma_x^2 & \sigma_{xy} & \sigma_{x\theta} \\ \sigma_{xy} & \sigma_y^2 & \sigma_{y\theta} \\ \sigma_{x\theta} & \sigma_{y\theta} & \sigma_\theta^2 \end{bmatrix}$$

Simplified diagonal form:

$$\mathbf{P} = \begin{bmatrix} P_{xx} & 0 & 0 \\ 0 & P_{yy} & 0 \\ 0 & 0 & P_{\theta\theta} \end{bmatrix}$$

### Prediction Step

#### State Prediction

$$\hat{\mathbf{x}} = f(\mathbf{x}, \mathbf{u}) = \begin{bmatrix} x + dx \\ y + dy \\ \text{normalize}(\theta + d\theta) \end{bmatrix}$$

#### Covariance Prediction

Full form:
$$\mathbf{P} \leftarrow \mathbf{F} \mathbf{P} \mathbf{F}^T + \mathbf{Q}$$

Simplified diagonal form:
$$P_{xx} \leftarrow P_{xx} + Q_x$$
$$P_{yy} \leftarrow P_{yy} + Q_y$$
$$P_{\theta\theta} \leftarrow P_{\theta\theta} + Q_\theta$$

Process noise parameters:

| Parameter | Default |
|-----------|---------|
| $Q_x$ | 0.01 m² |
| $Q_y$ | 0.01 m² |
| $Q_\theta$ | 0.005 rad² |

### IMU Update

#### Innovation

$$\tilde{y}_\theta = \text{normalize}(\theta_{IMU} - \hat{\theta})$$

#### Innovation Covariance

$$S_\theta = P_{\theta\theta} + R_\theta$$

#### Kalman Gain

$$K_\theta = \frac{P_{\theta\theta}}{S_\theta}$$

#### State Update

$$\theta \leftarrow \text{normalize}(\hat{\theta} + K_\theta \cdot \tilde{y}_\theta)$$

#### Covariance Update

$$P_{\theta\theta} \leftarrow (1 - K_\theta) \cdot P_{\theta\theta}$$

### UWB Update

#### Measurement Model

$$\mathbf{z} = \begin{bmatrix} z_x \\ z_y \end{bmatrix} = \begin{bmatrix} x \\ y \end{bmatrix} + \mathbf{v}$$

#### Innovation

$$\tilde{y}_x = z_x - \hat{x}$$
$$\tilde{y}_y = z_y - \hat{y}$$

#### Innovation Covariance

$$S_x = P_{xx} + R_x$$
$$S_y = P_{yy} + R_y$$

Measurement noise: $R_x = R_y = 0.15$ m²

#### Kalman Gain

$$K_x = \frac{P_{xx}}{S_x}$$
$$K_y = \frac{P_{yy}}{S_y}$$

#### State Update

$$x \leftarrow \hat{x} + K_x \cdot \tilde{y}_x$$
$$y \leftarrow \hat{y} + K_y \cdot \tilde{y}_y$$

#### Covariance Update

$$P_{xx} \leftarrow (1 - K_x) \cdot P_{xx}$$
$$P_{yy} \leftarrow (1 - K_y) \cdot P_{yy}$$

---

## Comparison

### Feature Matrix

| Feature | Tier 1 | Tier 2 | Tier 3 |
|---------|--------|--------|--------|
| Position accuracy | Unbounded drift | Unbounded drift | Bounded |
| Heading accuracy | Unbounded drift | Bounded | Bounded |
| Computational cost | O(1) | O(1) | O(1) |
| Sensors | Encoders | Encoders + IMU | Encoders + IMU + UWB |
| Uncertainty estimate | No | No | Yes |

### Expected Accuracy

| Tier | Position Error (10m) | Heading Error |
|------|----------------------|---------------|
| 1 | ~50-100 cm | ~5-10° |
| 2 | ~50-100 cm | ~1-2° |
| 3 | ~5-15 cm | ~1-2° |

---

## References

1. Siegwart, R., Nourbakhsh, I. R., & Scaramuzza, D. (2011). *Introduction to Autonomous Mobile Robots*. MIT Press.

2. Thrun, S., Burgard, W., & Fox, D. (2005). *Probabilistic Robotics*. MIT Press.

3. Taheri, H., Qiao, B., & Ghaeminezhad, N. (2015). "Kinematic model of a four mecanum wheeled mobile robot." *International Journal of Computer Applications*.

4. Kalman, R. E. (1960). "A New Approach to Linear Filtering and Prediction Problems." *Journal of Basic Engineering*.
