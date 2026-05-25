# mavtech_filter

ROS2 C++ modular ESEKF for quadrotor state estimation in NED frame.

Initial sensor setup:

- VIO odometry in NED at ~10 Hz
- 6-axis IMU at ~200 Hz
- Fused odometry output in NED at configurable rate, default 50 Hz

## State

Nominal state:

```text
p_WB, v_WB, q_WB, b_g, b_a
```

Error state:

```text
dp, dv, dtheta, dbg, dba
```

The node assumes:

- world frame is NED
- `q_WB` maps body-frame vectors into world/NED
- gravity is `[0, 0, +9.80665]` m/s²
- raw IMU vectors are transformed through:

```cpp
R_imu_to_vio_ned =
  [ 0, 1,  0
    1, 0,  0
    0, 0, -1 ]
```

## Build

```bash
cd ~/ros_ws/src
cp -r /path/to/mavtech_filter .
cd ~/ros_ws
colcon build --symlink-install --packages-select mavtech_filter
source install/setup.bash
```

## Run

```bash
ros2 launch mavtech_filter mavtech_filter.launch.py
```

or with custom config:

```bash
ros2 launch mavtech_filter mavtech_filter.launch.py config_file:=/path/to/mavtech_filter.yaml
```

## Important tuning notes

Start with `use_vio_velocity: false` unless your VIO twist is confirmed to be in the same NED/world convention.

For aggressive flight, increase process noise first:

- `accel_noise_density`
- `gyro_noise_density`

If the filter follows VIO too loosely, decrease:

- `vio_pos_std`
- `vio_ori_std_rad`

If the fused output is too noisy or VIO jumps are too strongly injected, increase those VIO standard deviations.
