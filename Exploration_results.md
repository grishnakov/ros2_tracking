Here is a thorough breakdown of everything found in `sample_container`.

---

## Overall Architecture

The workspace at `/home/projects/ros2_ws/src/ucsd_robocar_hub2` is the **UCSD Robocar Hub2** — a modular ROS2 (Foxy) framework for small autonomous vehicles. It contains 7 packages arranged in layers:

```
[Sensors]          [Actuators]
   ↓                    ↑
[ucsd_robocar_sensor2_pkg]   [ucsd_robocar_actuator2_pkg]
   ↓                    ↑
[ucsd_robocar_lane_detection2_pkg]   ← perception/vision
   ↓                    ↑
[ucsd_robocar_control2_pkg]          ← PID/LQR/LQG/MPC
   ↓                    ↑
[ucsd_robocar_path2_pkg]             ← path planning
         ↕
[ucsd_robocar_nav2_pkg]              ← orchestrator
```

---

## 1. The Config-Driven Launch System

This is the most important thing to understand. **Nothing is hardcoded** — the entire system is controlled by two YAML config files.

### Entry point: `all_nodes.launch.py`

```python
# Reads two config files and dynamically launches whatever is enabled
node_list_input_path    → node_config.yaml         (which algorithm nodes to run)
node_packages_info_path → node_pkg_locations_ucsd.yaml  (maps name → package + launch file)
```

**Step 1:** `node_config.yaml` — set `1` to enable, `0` to disable:

```yaml
# sensors/hardware
all_components: 0      # if 1, launches all_components.launch.py (hardware layer)

# camera navigation pipeline
camera_nav: 0          # if 1, launches lane_detection + lane_guidance nodes

# control algorithms
pid_launch: 0
lqr_launch: 0
lqg_launch: 0

# path following
tube_follower_launch: 0
curve_localizer_launch: 0

# basics (for debugging)
sub_camera_launch: 0
subpub_camera_actuator_launch: 0
```

**Step 2:** `node_pkg_locations_ucsd.yaml` — maps each name to a `[package, launch_file]`:

```yaml
camera_nav: ['ucsd_robocar_lane_detection2_pkg', 'camera_nav.launch.py']
pid_launch:  ['ucsd_robocar_control2_pkg', 'pid_launch.launch.py']
all_components: ['ucsd_robocar_nav2_pkg', 'all_components.launch.py']
```

### Hardware layer: `all_components.launch.py`

This is a **second** config-driven launcher, controlled by `car_config.yaml` and `pkg_locations_ucsd.yaml`:

**`car_config.yaml`** — hardware toggles:

```yaml
webcam: 0           # USB webcam on /dev/video0
intel: 0            # Intel RealSense (realsense2_camera package)
oakd: 0             # OAK-D camera (depthai-ros)
vesc_with_odom: 0   # VESC using f1tenth vesc_driver stack (AckermannDrive)
vesc_without_odom: 0 # VESC using PyVESC directly (Twist)
rp_lidar: 0
ld06: 0
ublox: 0            # u-blox GPS
artemis: 0          # IMU
```

**`pkg_locations_ucsd.yaml`** — maps hardware name → `[package, launch_file, type, topic]`:

```yaml
webcam:           ['ucsd_robocar_sensor2_pkg', 'camera_webcam.launch.py',  'camera', '/camera/color/image_raw']
vesc_without_odom:['ucsd_robocar_actuator2_pkg','vesc_twist.launch.py',    'motor',  '/cmd_vel']
vesc_with_odom:   ['ucsd_robocar_actuator2_pkg','vesc_odom.launch.py',     'motor',  '/ackermann_drive']
ublox:            ['ucsd_robocar_sensor2_pkg', 'gps_ublox.launch.py',      'gps',    '/gps_topic_name']
```

---

## 2. VESC Interface — Two Modes

### Mode A: `vesc_without_odom` (simple PyVESC, used for lane following)

**Launched by:** `vesc_twist.launch.py`  
**Node:** `vesc_twist_node` in `ucsd_robocar_actuator2_pkg`

**Subscribe to:** `/cmd_vel` (`geometry_msgs/Twist`)

| Field | Meaning | Range |
|---|---|---|
| `msg.linear.x` | throttle | `[-1, 1]` → scaled to RPM |
| `msg.angular.z` | steering | `[-1, 1]` → remapped to `[0, 1]` servo value |

**Under the hood** (`vesc_client.py`): uses `pyvesc` Python library directly over serial:
- Port: `/dev/ttyACM0`, baud: `115200`
- `v.set_rpm(rpm)` — controls motor speed
- `v.set_servo(angle)` — controls steering servo (0.0 = full right, 0.5 = straight, 1.0 = full left)

**Calibration file:** `vesc_twsit_calibration.yaml` (loaded by `vesc_twist.launch.py`) or `ros_racer_calibration.yaml` (loaded by the lane_detection launch):

```yaml
vesc_twist_node:
  ros__parameters:
    max_potential_rpm: 20000
    steering_polarity: 1       # flip to -1 if steering is reversed
    throttle_polarity: 1       # flip to -1 if motor is reversed
    zero_throttle: -0.032
    max_throttle: 0.382        # fraction of max_rpm
    min_throttle: 0.363
    max_right_steering: 0.792  # servo value for full right
    straight_steering: 0.0     # neutral (in [-1,1] space before remapping)
    max_left_steering: -0.831
```

### Mode B: `vesc_with_odom` (f1tenth vesc_driver stack)

**Launched by:** `vesc_odom.launch.py`  
**4 nodes from f1tenth repos** (`/home/projects/sensor2_ws/src/vesc/`):

- **`vesc_driver_node`** — serial comms with VESC on `/dev/ttyACM0`
- **`ackermann_to_vesc_node`** — converts `AckermannDriveStamped` → VESC erpm + servo
- **`vesc_to_odom_node`** — converts VESC state → `nav_msgs/Odometry` (on `/odom`)
- **`ackermann_mux_node`** — multiplexes joystick vs navigation commands by priority

**Subscribe to:** `AckermannDriveStamped` (on the `drive` or `teleop` mux input topics)

**Key parameters** (`vesc_odom.yaml`):
```yaml
speed_to_erpm_gain: 4921.82   # erpm = gain * speed_m/s + offset
speed_to_erpm_offset: 93.59
steering_angle_to_servo_gain: 1.2135
steering_angle_to_servo_offset: 0.4
port: /dev/ttyACM0
```

---

## 3. Camera and X11 Video Streaming

### Camera Node (`webcam_node.py`)

Uses OpenCV directly — **no ROS camera driver needed for a USB webcam**:

```python
self.cap = cv2.VideoCapture(0)   # opens /dev/video0 (mounted into container)
# Publishes sensor_msgs/Image at 30fps to:
CAMERA_TOPIC_NAME = '/camera/color/image_raw'
```

The container run command binds the camera device:
```
/dev/video0:/dev/video0  (from docker inspect)
```

### X11 Display Over SSH

The container is set up for X11 forwarding. From `docker inspect`:

```
Binds: /home/user/.Xauthority:/root/.Xauthority:rw
Env:   DISPLAY=   (empty — you must set it before launching GUI nodes)
```

**How it works:**
1. On your host machine, SSH in with `ssh -X user@host` (or `ssh -Y`)
2. The `DISPLAY` variable (e.g., `DISPLAY=:10.0`) is forwarded to the host
3. Inside the container, `DISPLAY=` is empty in `.bashrc` — **you must set it manually**:

```bash
# Inside sample_container, before running anything with GUI:
export DISPLAY=:0    # or whatever your host's $DISPLAY is
```

4. The Xauthority file is already mounted, so X11 auth works automatically

### Debug CV windows

The `lane_detection_node` has a `debug_cv` parameter. When set to `1`, it calls `cv2.imshow()` which opens an X11 window showing the processed camera feed in real time:

```yaml
# In ros_racer_calibration.yaml or via ros2 param set:
lane_detection_node:
  ros__parameters:
    debug_cv: 1   # set to 1 to enable live OpenCV windows
```

To enable at runtime:
```bash
ros2 param set /lane_detection_node debug_cv 1
```

---

## 4. The Lane Following Pipeline (Camera → VESC)

When `camera_nav: 1` is set in `node_config.yaml` and the appropriate hardware is enabled, this pipeline runs:

```
/dev/video0
    ↓ (cv2.VideoCapture)
webcam_node
    ↓ publishes sensor_msgs/Image
/camera/color/image_raw
    ↓ subscribes
lane_detection_node
    (HSV color filter → contour detection → centroid error calculation)
    ↓ publishes std_msgs/Float32  (error in [-1, 1])
/centroid
    ↓ subscribes
lane_guidance_node
    (PID controller on centroid error → throttle gain scheduling)
    ↓ publishes geometry_msgs/Twist
/cmd_vel
    ↓ subscribes
vesc_twist_node
    (remap angular.z to [0,1] servo, scale linear.x to RPM)
    ↓ PyVESC serial
/dev/ttyACM0 → VESC → motor + servo
```

---

## 5. Environment Setup (Critical for Your Container)

The `source_ros2_pkg()` function in `.bashrc` shows what must be sourced before `colcon build` or `ros2 launch`:

```bash
source /opt/ros/foxy/setup.bash
source /home/projects/sensor2_ws/src/cameras/oakd/install/setup.bash
source /home/projects/sensor2_ws/src/imu/artemis_openlog/install/setup.bash
source /home/projects/sensor2_ws/src/vesc/install/setup.bash           # ← critical for VESC mode B
source /home/projects/sensor2_ws/src/lidars/ld06/ros2/install/setup.bash
source /home/projects/rosboard_ws/install/setup.bash
```

Also, `ROS_DOMAIN_ID=96` is exported. **Your `ros2_doc` container must use the same domain ID** to see topics from `sample_container`:
```bash
export ROS_DOMAIN_ID=96
```

---

## 6. GPS — u-blox + robot_localization (and Point One Nav)

The sample container uses a **u-blox GPS** driver (`KumarRobotics/ublox`, foxy-devel branch), installed in `/home/projects/sensor2_ws/src/gps/ublox/`. Enable it with `ublox: 1` in `car_config.yaml`.

The reference in `pkg_locations_ucsd.yaml` points to `gps_ublox.launch.py` which does not yet exist in the sensor package — you would need to create it, similar to the other sensor launch files, pointing to the ublox_node executable.

**For Point One Nav (Polaris RTK GNSS):**  
There is **no Point One Nav package in this container**. The recommended approach is:

1. Install the [Point One Nav ROS2 driver](https://github.com/PointOneNav/ros2-fusion-engine-driver) — it publishes `sensor_msgs/NavSatFix` to `/gnss/fix` and optionally `geometry_msgs/PoseWithCovarianceStamped`
2. The `/home/projects/localization2_ws/` workspace already has `robot_localization` built with `navsat_transform_node`
3. The example at `/home/projects/localization2_ws/install/robot_localization/share/robot_localization/launch/dual_ekf_navsat_example.launch.py` shows the full fusion setup:
   - `ekf_filter_node_odom` → fuses wheel odometry (VESC odom) + IMU → `/odometry/local`
   - `ekf_filter_node_map` → fuses local odometry + GPS → `/odometry/global`
   - `navsat_transform_node` → subscribes to `gps/fix` + `imu/data`, publishes `odometry/gps` for the map EKF

The key remapping: Point One Nav's `NavSatFix` topic → remap to `gps/fix` for the `navsat_transform_node`.

---

## Summary: What to Enable in Your Package

To replicate the core functionality in your `ros2_doc` container:

| Feature | `car_config.yaml` | `node_config.yaml` |
|---|---|---|
| USB camera | `webcam: 1` | — |
| VESC (simple) | `vesc_without_odom: 1` | — |
| VESC (with odom) | `vesc_with_odom: 1` | — |
| Lane following | `webcam: 1`, `vesc_without_odom: 1` | `all_components: 1`, `camera_nav: 1` |
| GPS | `ublox: 1` | — |

The key insight: **you do not write node startup code** — you set flags in the two YAML config files, and `all_nodes.launch.py` wires everything together. To add your own package, add an entry to `node_pkg_locations_ucsd.yaml` and set it to `1` in `node_config.yaml`.