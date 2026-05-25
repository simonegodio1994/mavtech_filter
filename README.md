# mavtech_filter

ROS2 C++ node for generating a high-rate NED odometry estimate from a low-rate VIO odometry source and a high-rate 6-axis IMU.

The current implementation is designed for this initial sensor setup:

- **VIO odometry** in NED frame, approximately 10 Hz.
- **IMU** at approximately 200 Hz.
- **Output odometry** in NED frame, typically 50 Hz or higher.

The filter is intentionally **VIO-anchored**. It does not try to replace the VIO trajectory with a free inertial navigation solution. Instead, every VIO odometry message becomes a trusted low-rate anchor, and the IMU is used only for short-horizon prediction between VIO updates.

This design is especially suitable for a quadrotor where the VIO estimate is the main localization source, but the output topic must be faster, smoother, and slightly forward-predicted for control or PX4/offboard integration.

---

## 1. Frames and conventions

The node assumes that all published output is expressed in a **NED world frame**:

```text
x: North / forward-like axis depending on upstream convention
y: East / lateral axis depending on upstream convention
z: Down
```

The state convention is:

```text
p_WB: position of body B in world/NED frame W
v_WB: linear velocity of body B in world/NED frame W
q_WB: orientation from body frame B to world/NED frame W
```

Gravity is therefore represented as:

```text
g_W = [0, 0, +9.80665] m/s²
```

The IMU is assumed to be a 6-axis IMU. The raw IMU vectors are transformed before being used by the predictor.

The transform currently used in the code is:

```cpp
R_imu_to_vio_ned =
  [ 0, 1,  0
    1, 0,  0
    0, 0, -1 ]
```

and, equivalently, the quaternion originally provided was:

```cpp
q_imu_to_vio_ned = Eigen::Quaterniond(
    0.0,
    M_SQRT1_2,
    M_SQRT1_2,
    0.0);
```

In the code, the matrix transform is used to transform both gyroscope and accelerometer vectors:

```cpp
omega_body = R_imu_to_vio_ned * omega_raw;
accel_body = R_imu_to_vio_ned * accel_raw;
```

---

## 2. High-level estimator design

The implemented estimator is best described as a:

```text
VIO-anchored short-horizon IMU predictor
```

Rather than a fully free-running ESEKF, the current implementation uses this structure:

```text
VIO update:
  anchor = {p_vio, v_vio, q_vio, t_vio}

IMU buffer:
  stores recent IMU samples

Publisher timer:
  predicts state from the latest VIO anchor to the latest IMU time,
  optionally plus an additional prediction lead.
```

This means:

1. The VIO remains the absolute reference.
2. The output frequency is independent from the VIO frequency.
3. The IMU only predicts over a short interval, normally tens of milliseconds.
4. Drift is bounded because every new VIO message resets the anchor.

The approach avoids long-term inertial drift while still producing a high-rate odometry stream.

---

## 3. Why not a fully free ESEKF?

A classical ESEKF would propagate position, velocity, attitude, accelerometer bias, and gyroscope bias continuously using the IMU, then correct with VIO as a measurement.

That approach is mathematically valid, but for this specific use case it caused undesirable behavior:

- Vertical drift from accelerometer integration.
- Sensitivity to gravity alignment.
- Corrections that did not follow the VIO tightly enough.
- Output that could deviate from OKVIS even though OKVIS should remain the main source.

The selected design is therefore more pragmatic:

```text
Use VIO as the trusted low-rate state.
Use IMU only for bounded short-term propagation.
```

This gives a signal that follows the VIO closely while reducing delay and increasing the output rate.

---

## 4. State representation

The nominal state used by the predictor is:

```cpp
struct State
{
  Eigen::Vector3d p_WB;
  Eigen::Vector3d v_WB;
  Eigen::Quaterniond q_WB;
};
```

The node does not currently estimate IMU biases online. Bias estimation could be added later if the estimator is upgraded to a full ESEKF with covariance propagation.

The current design avoids bias estimation because the IMU is integrated only over a short prediction horizon.

---

## 5. Input topics

### 5.1 IMU input

Type:

```text
sensor_msgs/msg/Imu
```

Typical topic:

```text
/imu/data
```

Expected rate:

```text
~200 Hz
```

QoS used:

```cpp
rclcpp::SensorDataQoS imu_qos;
imu_qos.keep_last(1500);
imu_qos.best_effort();
```

The IMU is stored in a rolling buffer. The buffer is used to integrate angular velocity and acceleration from the latest VIO anchor timestamp to the desired output timestamp.

---

### 5.2 VIO odometry input

Type:

```text
nav_msgs/msg/Odometry
```

Typical topic:

```text
/okvis/okvis_odometry_ned
```

Expected rate:

```text
~10 Hz
```

QoS used:

```cpp
rclcpp::QoS vio_odom_qos(rclcpp::KeepLast(1));
vio_odom_qos.best_effort();
vio_odom_qos.durability_volatile();
```

Each VIO odometry message provides a new anchor:

```text
p_anchor = vio.pose.position
q_anchor = vio.pose.orientation
v_anchor = vio.twist.linear, if enabled
```

If VIO twist is not trusted, velocity can be estimated from finite differences of consecutive VIO positions. However, for this implementation, using VIO twist directly is recommended if the twist is already stable and expressed in the correct NED/world convention.

---

## 6. Output topic

Type:

```text
nav_msgs/msg/Odometry
```

Typical topic:

```text
/mavtech_filter/odom_ned
```

QoS used:

```cpp
rclcpp::QoS fused_odom_qos(rclcpp::KeepLast(1));
fused_odom_qos.best_effort();
fused_odom_qos.durability_volatile();
```

The output is expressed in the configured NED world frame.

The output contains:

```text
pose.position    = predicted position in NED
pose.orientation = predicted body-to-NED orientation
twist.linear     = predicted linear velocity in NED
```

---

## 7. Prediction model

At every VIO update, the filter stores:

```text
p0, v0, q0, t0
```

At every output timer event, it selects a prediction target time:

```text
t_target = latest_imu_stamp + extra_prediction_s
```

The prediction horizon is limited:

```text
dt = clamp(t_target - t0, 0, max_prediction_horizon_s)
```

The predicted orientation is obtained by integrating the gyroscope:

```text
q_pred = q0 ⊗ Exp(∫ omega_imu dt)
```

The predicted velocity can include a bounded IMU acceleration contribution:

```text
v_pred = v0 + gain_v * clamp(∫ a_imu dt)
```

The predicted position can optionally include bounded IMU acceleration contribution:

```text
p_pred = p0 + v0 * dt + gain_p * clamp(∫∫ a_imu dt²)
```

However, for takeoff and flight tests, position prediction from accelerometer should usually remain disabled or very tightly bounded, because accelerometer bias and gravity projection errors can quickly create position drift.

---

## 8. Takeoff Vz ramp predictor

During takeoff, VIO twist often behaves like a low-rate sampled signal. If the output simply republishes the latest VIO twist, the vertical velocity appears as a staircase:

```text
Vz_vio: sample-and-hold at ~10 Hz
```

The latest code adds a VIO twist-ramp predictor to reduce this effect, especially for `Vz`.

It estimates a filtered acceleration from consecutive VIO twist samples:

```text
a_vio = (v_vio[k] - v_vio[k-1]) / dt_vio
a_vio_filtered = low_pass(a_vio)
```

Then predicts:

```text
v_out = v_vio + gain * a_vio_filtered * horizon
```

Usually this ramp predictor should be enabled for **Z only**:

```yaml
use_vio_twist_ramp_for_z_only: true
```

This keeps XY tightly attached to the VIO while improving the takeoff vertical velocity.

---

## 9. Important parameters

### 9.1 Topic and frame parameters

```yaml
imu_topic: "/imu/data"
vio_topic: "/okvis/okvis_odometry_ned"
odom_topic: "/mavtech_filter/odom_ned"

world_frame_id: "world_ned"
body_frame_id: "base_link"
```

Description:

| Parameter | Meaning |
|---|---|
| `imu_topic` | Input IMU topic. |
| `vio_topic` | Input VIO odometry topic. |
| `odom_topic` | Output predicted odometry topic. |
| `world_frame_id` | Output odometry world frame, expected to be NED. |
| `body_frame_id` | Output odometry child frame. |

---

### 9.2 Timing parameters

```yaml
publish_rate_hz: 50.0
vio_delay_s: 0.0
max_prediction_horizon_s: 0.12
extra_prediction_s: 0.0
imu_buffer_s: 2.0
```

| Parameter | Meaning |
|---|---|
| `publish_rate_hz` | Output odometry publication rate. |
| `vio_delay_s` | Optional correction if VIO header stamp represents publication time rather than measurement time. |
| `max_prediction_horizon_s` | Maximum total prediction horizon from the latest VIO anchor. |
| `extra_prediction_s` | Additional lead time added to the output prediction target. |
| `imu_buffer_s` | Duration of IMU samples kept in memory. |

Recommended usage:

- Use `vio_delay_s: 0.0` if OKVIS already timestamps odometry with the image/state time.
- Use `extra_prediction_s` if the output still appears delayed and you want an explicit forward prediction.
- Keep `max_prediction_horizon_s` conservative to avoid extrapolation far beyond the latest VIO anchor.

---

### 9.3 VIO velocity anchor parameters

```yaml
use_vio_twist_velocity: true
smooth_vio_anchor_velocity: false
vio_velocity_alpha: 1.0
```

| Parameter | Meaning |
|---|---|
| `use_vio_twist_velocity` | If true, uses `msg.twist.twist.linear` from VIO as anchor velocity. |
| `smooth_vio_anchor_velocity` | If true, low-pass filters anchor velocity between VIO updates. |
| `vio_velocity_alpha` | Smoothing coefficient for VIO velocity if smoothing is enabled. |

Recommended:

```yaml
use_vio_twist_velocity: true
smooth_vio_anchor_velocity: false
vio_velocity_alpha: 1.0
```

This avoids adding phase delay to the anchor velocity.

---

### 9.4 IMU orientation prediction

```yaml
use_imu_gyro_for_orientation_prediction: true
```

If enabled, the node integrates gyroscope measurements from the VIO anchor timestamp to the prediction target time.

This is usually safe over short horizons and improves attitude latency.

---

### 9.5 IMU acceleration prediction

```yaml
use_imu_accel_for_velocity_prediction: true
use_imu_accel_for_position_prediction: false
```

| Parameter | Meaning |
|---|---|
| `use_imu_accel_for_velocity_prediction` | Enables short-term velocity prediction using integrated IMU acceleration. |
| `use_imu_accel_for_position_prediction` | Enables short-term position prediction using double-integrated IMU acceleration. |

Recommended:

```yaml
use_imu_accel_for_velocity_prediction: true
use_imu_accel_for_position_prediction: false
```

Position prediction from acceleration should be enabled only if the IMU frame, gravity compensation, and accelerometer calibration are well validated.

---

### 9.6 IMU acceleration tuning

```yaml
imu_velocity_prediction_gain: 0.55
imu_velocity_prediction_gain_xy: 0.05
imu_velocity_prediction_gain_z: 0.15

max_imu_prediction_horizon_s: 0.060
max_imu_velocity_delta_xy_mps: 0.020
max_imu_velocity_delta_z_mps: 0.050

max_imu_position_delta_xy_m: 0.005
max_imu_position_delta_z_m: 0.010

imu_accel_lpf_tau_s: 0.080
imu_accel_deadband_mps2: 0.12
```

| Parameter | Meaning |
|---|---|
| `imu_velocity_prediction_gain` | Global gain applied to IMU velocity delta. |
| `imu_velocity_prediction_gain_xy` | Additional XY gain. Useful to reduce XY bias. |
| `imu_velocity_prediction_gain_z` | Additional Z gain. Useful for takeoff/landing. |
| `max_imu_prediction_horizon_s` | Maximum horizon over which IMU acceleration is integrated. |
| `max_imu_velocity_delta_xy_mps` | Clamp on IMU-induced XY velocity change. |
| `max_imu_velocity_delta_z_mps` | Clamp on IMU-induced Z velocity change. |
| `max_imu_position_delta_xy_m` | Clamp on IMU-induced XY position change. |
| `max_imu_position_delta_z_m` | Clamp on IMU-induced Z position change. |
| `imu_accel_lpf_tau_s` | Low-pass time constant for acceleration before integration. |
| `imu_accel_deadband_mps2` | Deadband for small residual acceleration. |

Tuning guidelines:

- If velocity is noisy, decrease gains and clamps.
- If velocity is delayed, increase `extra_prediction_s`, `max_imu_prediction_horizon_s`, or Z ramp parameters.
- If `Vy` develops offset, reduce `imu_velocity_prediction_gain_xy` or `max_imu_velocity_delta_xy_mps`.
- If takeoff `Vz` is too stair-stepped, increase VIO twist-ramp parameters rather than trusting raw IMU acceleration too much.

---

### 9.7 VIO twist-ramp prediction

```yaml
use_vio_twist_ramp_prediction: true
use_vio_twist_ramp_for_z_only: true
vio_twist_ramp_gain: 0.90
vio_twist_ramp_accel_lpf_tau_s: 0.22
max_vio_twist_ramp_horizon_s: 0.10
max_vio_twist_ramp_delta_xy_mps: 0.00
max_vio_twist_ramp_delta_z_mps: 0.18
```

| Parameter | Meaning |
|---|---|
| `use_vio_twist_ramp_prediction` | Enables twist-ramp prediction from VIO velocity trend. |
| `use_vio_twist_ramp_for_z_only` | Applies ramp prediction only to Z. Recommended for takeoff. |
| `vio_twist_ramp_gain` | Gain applied to filtered VIO acceleration trend. |
| `vio_twist_ramp_accel_lpf_tau_s` | Low-pass time constant for VIO acceleration trend. |
| `max_vio_twist_ramp_horizon_s` | Maximum ramp prediction horizon. |
| `max_vio_twist_ramp_delta_xy_mps` | Clamp on XY ramp velocity correction. |
| `max_vio_twist_ramp_delta_z_mps` | Clamp on Z ramp velocity correction. |

This is the preferred mechanism to improve takeoff `Vz` without overusing the accelerometer.

---

### 9.8 Output smoothing

```yaml
enable_pose_output_smoothing: false
enable_twist_output_smoothing: true
twist_output_smoothing_tau_s: 0.010
```

| Parameter | Meaning |
|---|---|
| `enable_pose_output_smoothing` | Smooths published pose. Can add delay. |
| `enable_twist_output_smoothing` | Smooths published twist. |
| `twist_output_smoothing_tau_s` | Time constant for twist smoothing. |

Recommended for low latency:

```yaml
enable_pose_output_smoothing: false
enable_twist_output_smoothing: true
twist_output_smoothing_tau_s: 0.010
```

Increase `twist_output_smoothing_tau_s` only if the twist is too jagged.

---

## 10. Recommended starting configuration

```yaml
esekf_node:
  ros__parameters:
    imu_topic: "/imu/data"
    vio_topic: "/okvis/okvis_odometry_ned"
    odom_topic: "/mavtech_filter/odom_ned"

    world_frame_id: "world_ned"
    body_frame_id: "base_link"

    publish_rate_hz: 50.0

    vio_delay_s: 0.0
    max_prediction_horizon_s: 0.12
    extra_prediction_s: 0.0
    imu_buffer_s: 2.0
    gravity_mps2: 9.80665

    use_vio_twist_velocity: true
    smooth_vio_anchor_velocity: false
    vio_velocity_alpha: 1.0

    use_imu_gyro_for_orientation_prediction: true

    use_imu_accel_for_position_prediction: false
    use_imu_accel_for_velocity_prediction: true

    imu_velocity_prediction_gain: 0.55
    imu_velocity_prediction_gain_xy: 0.05
    imu_velocity_prediction_gain_z: 0.15

    max_imu_prediction_horizon_s: 0.060
    max_imu_velocity_delta_xy_mps: 0.020
    max_imu_velocity_delta_z_mps: 0.050
    max_imu_position_delta_xy_m: 0.005
    max_imu_position_delta_z_m: 0.010

    imu_accel_lpf_tau_s: 0.080
    imu_accel_deadband_mps2: 0.12

    use_vio_twist_ramp_prediction: true
    use_vio_twist_ramp_for_z_only: true
    vio_twist_ramp_gain: 0.90
    vio_twist_ramp_accel_lpf_tau_s: 0.22
    max_vio_twist_ramp_horizon_s: 0.10
    max_vio_twist_ramp_delta_xy_mps: 0.00
    max_vio_twist_ramp_delta_z_mps: 0.18

    enable_pose_output_smoothing: false
    enable_twist_output_smoothing: true
    twist_output_smoothing_tau_s: 0.010

    max_output_position_error_m: 0.05
```

---

## 11. Tuning guide

### Output still delayed

Increase:

```yaml
extra_prediction_s: 0.030
```

or:

```yaml
extra_prediction_s: 0.050
max_prediction_horizon_s: 0.16
```

If the delay is mainly on `Vz`, increase:

```yaml
vio_twist_ramp_gain: 1.05
max_vio_twist_ramp_horizon_s: 0.14
max_vio_twist_ramp_delta_z_mps: 0.24
```

---

### Output too noisy

Increase:

```yaml
twist_output_smoothing_tau_s: 0.020
imu_accel_lpf_tau_s: 0.100
```

Decrease:

```yaml
imu_velocity_prediction_gain_z
max_imu_velocity_delta_z_mps
vio_twist_ramp_gain
```

---

### `Vy` offset appears

Reduce:

```yaml
imu_velocity_prediction_gain_xy: 0.0
max_imu_velocity_delta_xy_mps: 0.0
```

This disables most IMU velocity prediction on XY while keeping Z improvements.

---

### Takeoff `Vz` still stair-stepped

Increase:

```yaml
vio_twist_ramp_gain: 1.05
max_vio_twist_ramp_delta_z_mps: 0.24
max_vio_twist_ramp_horizon_s: 0.14
```

Reduce twist smoothing:

```yaml
twist_output_smoothing_tau_s: 0.006
```

---

### Position diverges from VIO

Keep:

```yaml
use_imu_accel_for_position_prediction: false
```

and reduce:

```yaml
max_imu_position_delta_xy_m
max_imu_position_delta_z_m
```

---

## 12. Adding an additional sensor

The current architecture can be extended in two ways:

1. **Anchor-style sensor update**
2. **Correction-style sensor update**

For this implementation, the recommended approach is to keep the VIO as the main anchor and use additional sensors to correct selected components of the anchor or prediction.

---

## 13. Example: adding a range finder

A downward range finder provides height above ground. In NED, depending on the map/world origin, it can constrain the vertical position or relative altitude.

Input example:

```text
sensor_msgs/msg/Range
```

Recommended integration:

```text
Use range finder to correct z position or vertical drift.
Do not replace full VIO pose.
Use it as a scalar measurement.
```

Measurement model:

```text
z_meas = h_range
z_pred = function(p_WB.z, terrain height, sensor extrinsic)
residual = z_meas - z_pred
```

Simple practical implementation:

```cpp
void rangeCallback(const sensor_msgs::msg::Range::SharedPtr msg)
{
  // 1. Convert range measurement to expected NED z correction.
  // 2. Check validity: min_range < range < max_range.
  // 3. Apply a bounded correction to anchor_.p_WB.z().
  // 4. Optionally correct anchor_.v_WB.z() using a low-pass derivative.
}
```

Recommended parameters:

```yaml
range_topic: "/range"
use_range_finder: true
range_z_gain: 0.2
max_range_z_correction_m: 0.05
```

This approach is useful during takeoff and landing when vertical VIO can be less reliable.

---

## 14. Example: adding optical flow

Optical flow usually provides image-plane motion. Combined with altitude, it can constrain horizontal velocity.

Input examples:

```text
geometry_msgs/msg/TwistStamped
custom optical_flow_msgs/msg/OpticalFlow
```

Recommended integration:

```text
Use optical flow as a body-frame horizontal velocity measurement.
Transform it into the NED/world frame using the current attitude.
Fuse it as a correction to v_WB.x and v_WB.y.
```

Measurement model:

```text
v_flow_body_xy = optical_flow_velocity_body
v_pred_body = R_WBᵀ * v_WB
residual_xy = v_flow_body_xy - v_pred_body_xy
```

Simple correction:

```cpp
void opticalFlowCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  Eigen::Vector2d v_flow_body(msg->twist.linear.x, msg->twist.linear.y);

  Eigen::Vector3d v_body_pred = anchor_.q_WB.conjugate() * anchor_.v_WB;

  Eigen::Vector2d residual = v_flow_body - v_body_pred.head<2>();

  // Apply bounded correction in body frame, then transform back to world.
  Eigen::Vector3d dv_body(residual.x(), residual.y(), 0.0);
  Eigen::Vector3d dv_world = anchor_.q_WB * dv_body;

  anchor_.v_WB += flow_gain * clampNorm(dv_world, max_flow_velocity_correction);
}
```

Recommended parameters:

```yaml
use_optical_flow: true
flow_velocity_gain: 0.2
max_flow_velocity_correction_mps: 0.15
```

Optical flow is especially useful in GPS-denied indoor flight where horizontal velocity observability is important.

---

## 15. Example: adding GPS

GPS provides global position and sometimes velocity. For an indoor/GPS-denied stack, GPS may be used only outdoors or as a fallback.

Input examples:

```text
sensor_msgs/msg/NavSatFix
nav_msgs/msg/Odometry
geometry_msgs/msg/TwistStamped
```

Recommended integration:

```text
Convert GPS to local NED.
Use GPS as a low-rate position correction.
Do not hard-reset the VIO anchor unless GPS quality is high.
```

Measurement model:

```text
p_gps_ned = converted GPS local position
residual = p_gps_ned - p_WB
```

Simple correction:

```cpp
void gpsOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  Eigen::Vector3d p_gps(
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z);

  Eigen::Vector3d residual = p_gps - anchor_.p_WB;

  if (residual.norm() < max_gps_correction_m) {
    anchor_.p_WB += gps_position_gain * residual;
  }
}
```

Recommended parameters:

```yaml
use_gps: true
gps_position_gain: 0.05
max_gps_correction_m: 1.0
gps_min_fix_quality: 3
```

GPS should be gated using covariance, fix quality, number of satellites, or innovation magnitude.

---

## 16. Example: adding wheel odometry

Wheel odometry can provide planar velocity. This is more relevant for ground robots, but the same framework can support it.

Input example:

```text
nav_msgs/msg/Odometry
```

Recommended integration:

```text
Use wheel odometry as a planar velocity correction.
Fuse only x/y velocity or body-frame forward velocity.
Do not use wheel z velocity.
```

Measurement model:

```text
v_wheel_body_x = measured forward velocity
v_pred_body = R_WBᵀ * v_WB
residual = v_wheel_body_x - v_pred_body.x
```

Simple correction:

```cpp
void wheelOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  double v_forward = msg->twist.twist.linear.x;

  Eigen::Vector3d v_body_pred = anchor_.q_WB.conjugate() * anchor_.v_WB;

  double residual = v_forward - v_body_pred.x();

  Eigen::Vector3d dv_body(wheel_gain * residual, 0.0, 0.0);
  Eigen::Vector3d dv_world = anchor_.q_WB * dv_body;

  anchor_.v_WB += dv_world;
}
```

Recommended parameters:

```yaml
use_wheel_odom: true
wheel_velocity_gain: 0.2
max_wheel_velocity_correction_mps: 0.2
```

---

## 17. Toward a full multi-sensor ESEKF

The current VIO-anchored predictor can be extended into a full ESEKF by adding:

```text
state:
  p, v, q, bg, ba

error-state:
  dp, dv, dtheta, dbg, dba

covariance:
  P ∈ R15x15
```

IMU propagation would update both the nominal state and covariance:

```text
x_k+1 = f(x_k, imu)
P_k+1 = F P_k Fᵀ + G Q Gᵀ
```

Each sensor would implement:

```text
residual r = z - h(x)
Jacobian H = ∂h/∂x
Kalman gain K = P Hᵀ (H P Hᵀ + R)⁻¹
error update dx = K r
inject dx into nominal state
```

This is the correct long-term architecture if the node must become a general multi-sensor estimator. However, for the current objective—high-rate, low-latency odometry that follows VIO tightly—the VIO-anchored predictor is simpler and safer.

---

## 18. Practical debugging checklist

### Check timestamps

```bash
ros2 topic echo /okvis/okvis_odometry_ned/header
ros2 topic echo /imu/data/header
```

Confirm whether VIO header stamps correspond to image/state time or publication time.

---

### Check topic rates

```bash
ros2 topic hz /okvis/okvis_odometry_ned
ros2 topic hz /imu/data
ros2 topic hz /mavtech_filter/odom_ned
```

Expected:

```text
VIO:    ~10 Hz
IMU:    ~200 Hz
Output: ~50 Hz
```

---

### Check frame consistency

Plot or print:

```text
roll, pitch, yaw
linear velocity x/y/z
position x/y/z
```

If one axis consistently shows offset, reduce IMU gain or clamp on that axis.

---

### Check takeoff behavior

For takeoff `Vz`, compare:

```text
/okvis/okvis_odometry_ned/twist/twist/linear/z
/mavtech_filter/odom_ned/twist/twist/linear/z
```

Tune in this order:

1. `vio_twist_ramp_gain`
2. `max_vio_twist_ramp_delta_z_mps`
3. `max_vio_twist_ramp_horizon_s`
4. `twist_output_smoothing_tau_s`
5. `extra_prediction_s`

---

## 19. Known limitations

The current implementation:

- Does not estimate accelerometer or gyroscope bias online.
- Does not perform full covariance propagation.
- Assumes VIO is the main trusted localization source.
- Uses bounded short-term IMU prediction rather than long-term inertial navigation.
- Requires correct frame convention for IMU and VIO.
- Can still show stair-step behavior if VIO twist itself is heavily quantized or low-rate.
- May need axis-specific tuning, especially for vertical takeoff dynamics.

---

## 20. Summary

The node implements a VIO-anchored high-rate odometry predictor:

```text
Low-rate VIO gives accurate pose/velocity anchors.
High-rate IMU predicts attitude and short-term dynamics.
Optional VIO twist-ramp prediction improves takeoff Vz.
Output is published at high rate in NED.
```

This provides a practical compromise between:

```text
VIO accuracy
IMU responsiveness
bounded drift
low latency
high-rate odometry output
```