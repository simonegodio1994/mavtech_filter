# Mathematical Approach of the MavTech VIO-Anchored IMU Predictor

## 1. Objective

The estimator generates a high-rate NED odometry output from:

- a low-rate VIO odometry source, approximately 10 Hz;
- a high-rate 6-axis IMU, approximately 200 Hz.

The estimator is not intended to behave as a free-running inertial navigation system. Instead, it is a **VIO-anchored short-horizon predictor**:

```text
VIO provides the trusted low-rate anchor.
IMU provides short-term propagation between VIO anchors.
The output is published at high rate and may be slightly forward-predicted.
```

The main design goal is:

```text
Follow the VIO trajectory tightly,
increase output rate,
reduce apparent delay,
avoid long-term IMU drift.
```

---

## 2. Frames and state convention

The output frame is assumed to be NED:

```text
W = NED world frame
B = body frame
```

The state stored at a VIO anchor is:

```text
x = { p_WB, v_WB, q_WB }
```

where:

```text
p_WB ∈ R³    position of the body in world/NED frame
v_WB ∈ R³    linear velocity of the body in world/NED frame
q_WB ∈ SO(3) body-to-world orientation quaternion
```

Gravity in NED is:

```text
g_W = [0, 0, +g]ᵀ,       g = 9.80665 m/s²
```

The IMU raw vectors are transformed into the VIO/NED-aligned body convention by:

```text
ω_B = R_imu_to_vio_ned · ω_raw
a_B = R_imu_to_vio_ned · a_raw
```

with:

```text
R_imu_to_vio_ned =
[ 0  1  0
  1  0  0
  0  0 -1 ]
```

---

## 3. VIO anchor model

Every VIO odometry message defines a new anchor:

```text
t₀ = t_vio
p₀ = p_vio(t₀)
q₀ = q_vio(t₀)
v₀ = v_vio(t₀)
```

If VIO twist is trusted:

```text
v₀ = twist_vio.linear
```

If VIO twist is not trusted, an alternative finite-difference velocity can be used:

```text
v₀ = (p_vio,k - p_vio,k-1) / (t_vio,k - t_vio,k-1)
```

In the current recommended configuration, the VIO twist is used directly and not smoothed:

```text
smooth_vio_anchor_velocity = false
vio_velocity_alpha = 1.0
```

This avoids introducing phase lag at the anchor level.

---

## 4. Prediction target time

At each output timer event, the prediction target is:

```text
t_target = t_latest_imu + t_extra
```

where:

```text
t_extra = extra_prediction_s
```

The prediction horizon is:

```text
Δt = clamp(t_target - t₀, 0, Δt_max)
```

with:

```text
Δt_max = max_prediction_horizon_s
```

This means the output can be predicted up to the latest available IMU timestamp, optionally plus an explicit lead time.

---

## 5. Gyroscope-based attitude propagation

The angular velocity is integrated from the anchor time to the target time.

For each IMU sample interval:

```text
δθ_k = ω_B,k · Δt_k
δq_k = Exp(δθ_k)
```

The quaternion propagation is:

```text
q_{k+1} = q_k ⊗ δq_k
```

Starting from:

```text
q(t₀) = q₀
```

the predicted orientation is:

```text
q_pred = q₀ ⊗ Π_k Exp(ω_B,k Δt_k)
```

This is safe over short horizons because gyro integration drift remains bounded by the next VIO anchor.

---

## 6. Acceleration model

The raw accelerometer vector is first transformed into the aligned body frame:

```text
a_B = R_imu_to_vio_ned · a_raw
```

It is then rotated into the world/NED frame and gravity-compensated:

```text
a_W = R(q_WB) · a_B + g_W
```

where:

```text
R(q_WB) = rotation matrix corresponding to q_WB
```

A low-pass filter is optionally applied:

```text
a_f,k = a_f,k-1 + α_k (a_W,k - a_f,k-1)
```

with:

```text
α_k = 1 - exp(-Δt_k / τ_a)
```

where:

```text
τ_a = imu_accel_lpf_tau_s
```

A deadband is applied component-wise:

```text
if |a_f,i| < a_deadband, then a_f,i = 0
```

This prevents very small residual accelerations from creating artificial velocity drift.

---

## 7. IMU velocity prediction

The IMU contribution to velocity is:

```text
Δv_imu = ∫_{t₀}^{t_target} a_f(t) dt
```

In discrete form:

```text
Δv_imu ≈ Σ_k a_f,k Δt_k
```

This contribution is bounded per axis:

```text
Δv_x = clamp(Δv_x, -Δv_xy,max, +Δv_xy,max)
Δv_y = clamp(Δv_y, -Δv_xy,max, +Δv_xy,max)
Δv_z = clamp(Δv_z, -Δv_z,max,  +Δv_z,max)
```

The velocity predicted from the anchor is:

```text
v_pred = v₀ + G_imu Δv_imu
```

with axis-specific gains:

```text
G_imu =
diag(g_xy, g_xy, g_z)
```

In the implementation, this appears as:

```text
v_pred,x = v₀,x + imu_velocity_prediction_gain · imu_velocity_prediction_gain_xy · Δv_x
v_pred,y = v₀,y + imu_velocity_prediction_gain · imu_velocity_prediction_gain_xy · Δv_y
v_pred,z = v₀,z + imu_velocity_prediction_gain · imu_velocity_prediction_gain_z  · Δv_z
```

The reason for axis-specific gains is practical: on the tested platform, XY accelerometer prediction tends to introduce more bias, while Z benefits more during takeoff and landing.

---

## 8. Optional IMU position prediction

The pure kinematic model would predict position as:

```text
p_pred = p₀ + v₀ Δt + ∫∫ a_f(t) dt²
```

Discretely:

```text
Δp_imu ≈ Σ_k [ v_imu,k Δt_k + 1/2 a_f,k Δt_k² ]
```

The implemented approach keeps this term optional and bounded:

```text
p_pred = p₀ + v₀ Δt + G_p clamp(Δp_imu)
```

where:

```text
G_p = imu_position_prediction_gain
```

In most flight tests, this should remain disabled or tightly limited:

```text
use_imu_accel_for_position_prediction = false
```

because double integration of accelerometer data is highly sensitive to:

- accelerometer bias;
- gravity alignment error;
- frame convention errors;
- vibration.

The VIO pose should remain the dominant position source.

---

## 9. VIO twist-ramp predictor

The most visible issue during takeoff is often a staircase-like vertical velocity:

```text
VIO twist is low-rate, usually around 10 Hz.
The published high-rate velocity may become sample-and-hold.
```

To improve this, the estimator computes a short-term ramp based on the trend of the VIO twist.

First, estimate the VIO acceleration from consecutive VIO velocity anchors:

```text
a_vio,k = (v_vio,k - v_vio,k-1) / (t_vio,k - t_vio,k-1)
```

Then low-pass filter it:

```text
a_vio,f,k = a_vio,f,k-1 + β_k (a_vio,k - a_vio,f,k-1)
```

with:

```text
β_k = 1 - exp(-Δt_vio / τ_vio)
```

where:

```text
τ_vio = vio_twist_ramp_accel_lpf_tau_s
```

The ramp velocity correction is:

```text
Δv_ramp = g_ramp · a_vio,f · h
```

where:

```text
g_ramp = vio_twist_ramp_gain
h = clamp(prediction_horizon, 0, max_vio_twist_ramp_horizon_s)
```

The correction is bounded:

```text
Δv_ramp,z = clamp(Δv_ramp,z, -Δv_ramp,z,max, +Δv_ramp,z,max)
```

Usually the ramp predictor is applied only on Z:

```text
use_vio_twist_ramp_for_z_only = true
```

so that:

```text
Δv_ramp,x = 0
Δv_ramp,y = 0
```

This gives the final predicted velocity:

```text
v_out = v₀ + G_imu Δv_imu + Δv_ramp
```

This is the main mechanism used to improve takeoff vertical velocity without relying too heavily on raw IMU acceleration.

---

## 10. Final pose and twist output

The output odometry is:

```text
pose.position    = p_out
pose.orientation = q_out
twist.linear     = v_out
```

where typically:

```text
q_out = q_pred
v_out = v₀ + G_imu Δv_imu + Δv_ramp
p_out = p₀ + v₀ Δt
```

or, if position IMU prediction is enabled:

```text
p_out = p₀ + v₀ Δt + G_p Δp_imu
```

The output remains bounded around the VIO because each new VIO message resets the anchor:

```text
p₀, v₀, q₀ ← latest VIO
```

Thus the estimator does not accumulate long-term inertial drift.

---

## 11. Relation to a classical ESEKF

A classical ESEKF would define a nominal state:

```text
x = [ p, v, q, b_g, b_a ]
```

and an error state:

```text
δx = [ δp, δv, δθ, δb_g, δb_a ]
```

The IMU propagation would be:

```text
ṗ = v
v̇ = R(q)(a_m - b_a - n_a) + g
q̇ = 1/2 q ⊗ (ω_m - b_g - n_g)
ḃ_g = n_bg
ḃ_a = n_ba
```

The covariance would propagate as:

```text
P_{k+1} = F_k P_k F_kᵀ + G_k Q_k G_kᵀ
```

A VIO update would use a residual:

```text
r = z_vio - h(x)
```

and a Kalman correction:

```text
K = P Hᵀ (H P Hᵀ + R)⁻¹
δx = K r
```

Then the nominal state would be corrected:

```text
p ← p + δp
v ← v + δv
q ← Exp(δθ) ⊗ q
b_g ← b_g + δb_g
b_a ← b_a + δb_a
```

The current implementation does not propagate the covariance and does not estimate IMU biases. It uses the VIO anchor as a deterministic correction:

```text
x_anchor ← x_vio
```

This is less general than a full ESEKF but more robust for the current objective:

```text
produce a high-rate, low-latency odometry topic that follows VIO tightly.
```

---

## 12. Why the current approach is appropriate

The current estimator is appropriate when:

```text
VIO is the trusted localization source.
IMU is reliable only over short intervals.
Long-term inertial drift must be avoided.
The controller needs a high-rate odometry topic.
```

The design avoids:

- long-term acceleration integration;
- large deviations from VIO;
- sensitivity to unestimated accelerometer bias;
- full covariance tuning complexity.

It provides:

- high-rate output;
- bounded delay compensation;
- short-term attitude prediction;
- short-term velocity prediction;
- improved vertical takeoff behavior through VIO twist-ramp prediction.

---

## 13. Compact mathematical summary

At each VIO update:

```text
x₀ = {p₀, v₀, q₀}
```

At each output time:

```text
Δt = clamp(t_target - t₀, 0, Δt_max)
```

Gyro propagation:

```text
q_out = q₀ ⊗ Π_k Exp(ω_B,k Δt_k)
```

Acceleration integration:

```text
a_W,k = R(q_k) a_B,k + g_W
Δv_imu = Σ_k LPF(a_W,k) Δt_k
```

VIO twist-ramp:

```text
a_vio = LPF((v_vio,k - v_vio,k-1) / Δt_vio)
Δv_ramp = clamp(g_ramp a_vio h)
```

Velocity output:

```text
v_out = v₀ + G_imu clamp(Δv_imu) + Δv_ramp
```

Position output:

```text
p_out = p₀ + v₀ Δt
```

or, if enabled:

```text
p_out = p₀ + v₀ Δt + G_p clamp(Δp_imu)
```

Final odometry:

```text
odom_NED = {p_out, q_out, v_out}
```
