# Report an issue
Please reach out to the support team https://support.thinklucid.com/contact-support/ 
Or email to support@thinklucid.com

# arena_camera_ros2
Arena Camera driver for ROS2

# Note
- Please provide your feedback is welcomed at support@thinklucid.com or the repo issue page
      
# Requirements
- 1 
  - OS       : Linux (x64/amd64/arm64) (==22.04) 
  - ROS2     : Humble Hawksbill distro (installation steps in ros2_arena_setup.sh)
  - ArenaSDK and arena_api : https://thinklucid.com/downloads-hub/
- or 2 Docker
  - ArenaSDK and arena_api : https://thinklucid.com/downloads-hub/
  - build the docker image provided
  
# Getting Started
- clone repo or download release
    
    `git clone https://github.com/lucidvisionlabs/arena_camera_ros2.git`

- install ArenaSDK and arena_api
  - https://thinklucid.com/downloads-hub/

- install ROS2 and setup the environment 
    
    `cat arena_camera_ros2/ros2_arena_setup.sh` to view the script

    `cd arena_camera_ros2 ; sudo sh ros2_arena_setup.sh` installs ROS2 Humble Hawksbill distro

- build workspace and its dependencies

    `source /opt/ros/humble/setup.bash` if using a regular terminal

    `cd arena_camera_ros2/ros2_ws`
    
    `rosdep update`

    `rosdep install --from-paths src --ignore-src --rosdistro humble -r -y`

    `colcon build --symlink-install # build workspace for dev`

- install the build

    `. install/setup.bash`

# Explore

## Nodes

### arena_camera_node

The main node. Represents one LUCID camera. Executables: `start` and `trigger_image`.

---

## ROS Arguments

### Basic Camera Configuration

- **serial**
  - Serial number of the device to open.
  - If not provided, the first discovered camera is used.
  - Example: `-p serial:=904240001`

- **topic**
  - Topic the camera publishes images on.
  - Default: `/arena_camera_node/images`
  - Example: `-p topic:=/cam0/images`

- **camera_name**
  - Name given to the camera; used as the `frame_id` in the image message header.
  - Default: `arena_camera`
  - Example: `-p camera_name:=front_left`

- **width**
  - Width of the captured image in pixels.
  - Default: camera's default user profile value.

- **height**
  - Height of the captured image in pixels.
  - Default: camera's default user profile value.

- **pixelformat**
  - Pixel format of the published image.
  - Supported values: `rgb8`, `rgba8`, `rgb16`, `rgba16`, `bgr8`, `bgra8`, `bgr16`, `bgra16`,
    `mono8`, `mono16`, `bayer_rggb8`, `bayer_bggr8`, `bayer_gbrg8`, `bayer_grbg8`,
    `bayer_rggb16`, `bayer_bggr16`, `bayer_gbrg16`, `bayer_grbg16`, `yuv422`
  - Default: camera's current pixel format.

---

### Image Control

- **gain**
  - Analog gain applied to the sensor (double).
  - Default: camera's default value.

- **exposure_time**
  - Sensor exposure time in microseconds (double).
  - Large values increase latency before the image is published.
  - When using `trigger_mode`, it is strongly recommended to set this to a fixed value so triggered images arrive in a bounded time.
  - Default: camera's default value (auto-exposure active).

- **target_brightness**
  - Target brightness for the auto-exposure algorithm, integer in the range supported by the camera (typically 0–255).
  - Enables `ExposureAuto=Continuous` mode; mutually exclusive with a fixed `exposure_time`.
  - Default: not set (camera default).

- **gamma**
  - Gamma correction applied to the image (double). `1.0` = no correction; values < 1 brighten shadows.
  - Default: camera's default value.

- **white_balance_enable**
  - Master toggle for white balance correction (`BalanceWhiteEnable`).
  - When `false`, no white balance is applied regardless of `white_balance_auto`.
  - Only meaningful on color cameras; mono cameras will log a warning and continue.
  - Default: `true`

- **white_balance_auto**
  - Controls how white balance is adjusted (`BalanceWhiteAuto`). Only takes effect when `white_balance_enable` is `true`.
  - `Continuous` — camera continuously adjusts the Red/Green/Blue channel ratios as lighting conditions change.
  - `Once` — auto white balance runs once until it converges, then locks in those values.
  - `Off` — manual; channel ratios are frozen at whatever value was last set (camera default or a prior `Once` run).
  - Default: `"Continuous"`

---

### Frame Rate

- **frame_rate**
  - Free-run acquisition frame rate in Hz (double).
  - Only applies when `trigger_mode` is `false`.
  - Default: `30.0`

---

### Trigger Mode (single-camera software trigger)

- **trigger_mode**
  - When `true`, the camera waits for a software trigger before capturing each frame.
  - When `false` (default), the camera runs free at `frame_rate` Hz and continuously publishes.
  - Values: `true` / `false`

  **Free-run example:**
  ```
  ros2 run arena_camera_node start --ros-args -p qos_reliability:=reliable -p topic:=image
  ros2 run image_tools showimage
  ```

  **Software trigger example:**
  ```
  ros2 run arena_camera_node start --ros-args -p trigger_mode:=true -p exposure_time:=5000 -p topic:=image
  ros2 run arena_camera_node trigger_image   # triggers one frame
  ```

---

### Synchronized Multi-Camera Triggering (PTP + GigE Vision Action Commands)

These parameters enable frame-accurate synchronized capture across multiple cameras using IEEE 1588 PTP and GigE Vision scheduled action commands. All cameras must share the same `action_device_key`, `action_group_key`, and `action_group_mask`. Exactly one node is designated the action master and fires the broadcast action command; all other nodes are passive receivers.

- **action_master**
  - Set to `true` on exactly one node (the one that broadcasts the action command).
  - Only the master exposes the `/trigger_all` service.
  - Default: `false`

- **action_device_key**
  - Action command device key (integer). Must match on all cameras and the fire command.
  - Default: `1`

- **action_group_key**
  - Action command group key (integer). Must match on all cameras.
  - Default: `1`

- **action_group_mask**
  - Action command group mask (integer). Must match on all cameras.
  - Default: `1`

- **action_lead_time**
  - Seconds added ahead of the scheduled execution time (double).
  - Must exceed worst-case latch + network latency; increase if frames are dropped at high rates (keep below `1 / action_trigger_rate`).
  - Default: `0.05`

- **action_trigger_rate**
  - Continuous trigger rate in Hz (double). When `> 0`, the master fires action commands automatically at this rate.
  - When `0` (default), only manual `/trigger_all` calls fire the command.
  - Default: `0.0`

- **action_get_image_timeout**
  - Timeout in milliseconds for `GetImage()` in the action-mode acquisition loop (integer).
  - Timeouts between triggers are expected and swallowed silently.
  - Default: `1000`

- **ptp_domain**
  - PTP domain number (integer). Must match the grandmaster (e.g., RTK) and the switch.
  - `0` leaves the camera at its default; non-zero values are applied in a try/catch since not every LUCID model exposes a writable domain node.
  - Default: `0`

- **ptp_lock_timeout**
  - Maximum seconds to wait for `PtpStatus == "Slave"` before refusing to fire an action command (double).
  - Default: `60.0`

  **Synchronized multi-camera example (6 cameras, camera 0 is master):**
  ```
  # On each camera node (adjust serial, camera_name, topic, gev_scftd per camera):
  ros2 run arena_camera_node start --ros-args \
    -p serial:=<SN> -p trigger_mode:=true -p exposure_time:=5000 \
    -p action_master:=false \
    -p action_device_key:=1 -p action_group_key:=1 -p action_group_mask:=1 \
    -p gev_scftd:=<per_camera_offset_ns>

  # On the master node only, add:
    -p action_master:=true -p action_trigger_rate:=10.0

  # Or trigger one synchronized shot manually:
  ros2 service call /trigger_all std_srvs/srv/Trigger
  ```

---

### GigE Vision Transmission Shaping

These parameters affect packet transmission timing only — they do not affect capture, exposure, or the PTP-synchronized `header.stamp`.

- **gev_scpd**
  - GigE stream channel packet delay in nanoseconds (integer).
  - Spaces out packets within a single frame to reduce burst load on the network.
  - `0` = leave camera default.
  - Default: `0`

- **gev_scftd**
  - GigE stream channel frame transmission delay in nanoseconds (integer).
  - Staggers when each camera transmits its already-captured frame so synchronized frames from multiple cameras do not all arrive at the host simultaneously, which can overflow receive buffers at high frame rates.
  - Set a different value per camera (e.g., multiples of a fixed offset) in multi-camera setups.
  - `0` = leave camera default.
  - Default: `0`

- **mtu**
  - GigE Vision stream channel packet size (`GevSCPSPacketSize`) in bytes (integer).
  - Set to `9000` to enable jumbo frames (requires the NIC and switch to also be configured for jumbo frames at the OS level).
  - Default: `1500` (standard Ethernet MTU)

---

### QoS Settings

Make sure the publisher and any subscriber use matching QoS settings, otherwise images will be published but the subscriber will not receive them.

- **qos_history**
  - History policy for the image publisher.
  - Supported values: `system_default`, `keep_last`, `keep_all`, `unknown`
  - Default: `keep_last`

- **qos_history_depth**
  - Depth for `keep_last` history policy.
  - Default: `5`

- **qos_reliability**
  - Reliability policy for the image publisher.
  - Supported values: `system_default`, `reliable`, `best_effort`, `unknown`
  - Default: `best_effort`

  More about QoS: https://index.ros.org/doc/ros2/Concepts/About-Quality-of-Service-Settings/

---

## Services

- **`<node_name>/trigger_image`**
  - Triggers a single image from a camera running in software `trigger_mode`.
  - Run: `ros2 run arena_camera_node trigger_image`

- **`/trigger_all`** (action master only)
  - Fires one synchronized shot across all cameras via a scheduled GigE Vision action command.
  - Available only on the node launched with `action_master:=true`.
  - Run: `ros2 service call /trigger_all std_srvs/srv/Trigger`

---

## Full Example

```
ros2 run arena_camera_node start --ros-args \
  -p serial:=904240001 \
  -p topic:=/cam0/images \
  -p camera_name:=front_left \
  -p width:=1920 \
  -p height:=1080 \
  -p pixelformat:=rgb8 \
  -p gain:=10.0 \
  -p exposure_time:=5000 \
  -p target_brightness:=70 \
  -p gamma:=1.0 \
  -p white_balance_enable:=true \
  -p white_balance_auto:=Continuous \
  -p frame_rate:=30.0 \
  -p mtu:=9000 \
  -p qos_reliability:=reliable
```

---

## Explore Executables

`ros2 pkg executables | grep arena`

All executables can be run with:

`ros2 run <package_name> <executable_name>`

---

# Road map
- support windows
- add -h flag to nodes
- showimage node to view 2D and 3D images
- launch file
- camera_info
- access to nodemaps
- settings dump/read to/from file
- support two devices
