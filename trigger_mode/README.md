# arena_camera_node_trigger — PTP + Scheduled Action Command synchronized triggering

This folder contains a **copy** of the stock `arena_camera_node` ROS2 package,
renamed to **`arena_camera_node_trigger`** and extended with a manual,
**PTP-synchronized** triggering mode driven by **scheduled GigE Vision Action
Commands**. The original `arena_camera_node` package is left byte-for-byte
untouched; this package builds side-by-side with it.

> Nothing in this folder was built, run, or tested on the authoring machine
> (no ROS2 / colcon / Arena SDK / cameras). Build and verify **on the vehicle**
> using the checklist below. See `ASSUMPTIONS-AND-VERIFY.md` for every item that
> could not be confirmed against an installed SDK.

## Why this exists

The OXTS AV200 RTK is the **PTP grandmaster**; the switch passes PTP through;
all six LUCID cameras are **PTP slaves** (`PtpEnable=true`, `PtpSlaveOnly=true`,
already set by `set_nodes_ptp_()`). PTP alone only synchronizes **timestamps**,
not **captures**. To make all six cameras expose the *same instant*, each camera
is set to `TriggerSource=Action0` and a single "action master" node broadcasts a
**scheduled action command** with an execute time in the shared RTK/PTP timebase.
All cameras receive the same command and expose simultaneously.

This is gated behind a new `action_trigger_mode` parameter and does not change
the behavior of the stock free-run or software-`trigger_mode` paths.

## What changed vs. the stock package

- `package.xml`, `CMakeLists.txt`: package/project renamed to
  `arena_camera_node_trigger`. `CMakeLists.txt` also adds
  `find_package(Threads)` + links `Threads::Threads` to the `start` target for
  the action-mode acquisition thread.
- `src/ArenaCameraNode.h` / `src/ArenaCameraNode.cpp`:
  - New params (all with defaults; see below).
  - `set_nodes_action_trigger_mode_()`: `TriggerSelector=FrameStart`,
    `TriggerMode=On`, `TriggerSource=Action0`, `ActionUnconditionalMode=On`,
    `ActionSelector=0`, `ActionDeviceKey/GroupKey/GroupMask`. Disables free-run
    `AcquisitionFrameRateEnable` so the action command fully gates capture.
  - Action-mode acquisition runs in a background `std::thread`
    (`publish_images_action_`) so the node still spins/serves services; GetImage
    timeouts between triggers are swallowed quietly.
  - `fire_scheduled_action_command_()` (action master): `PtpDataSetLatch` →
    `PtpDataSetLatchValue` → compute execute time → set the `ActionCommand*`
    system nodes (broadcast `0xFFFFFFFF`) → `ActionCommandFireCommand`.
  - `/trigger_all` (`std_srvs/srv/Trigger`) one-shot service (master only) and an
    optional continuous timer (`action_trigger_rate`).
  - `wait_for_ptp_lock_()`: refuses to fire until this device's `PtpStatus`
    settles on `Slave`.
  - Optional `ptp_domain` (applied only when non-zero, in try/catch).
- All other source files are exact copies.

## Build (ON THE VEHICLE)

The renamed package has a distinct name, so colcon builds it alongside the stock
one. Copy the package into your colcon workspace `src/` and build just it:

```bash
# from the repo, copy the renamed package next to the original
cp -r arena_camera_ros2/trigger_mode/arena_camera_node_trigger \
      arena_camera_ros2/ros2_ws/src/

cd arena_camera_ros2/ros2_ws
colcon build --packages-select arena_camera_node_trigger
source install/setup.bash
```

(The original `arena_camera_ros2/ros2_ws` has a top-level `COLCON_IGNORE`; build
from wherever your working colcon workspace lives. The only requirement is that
`arena_camera_node_trigger` ends up under a workspace `src/`.)

## Launch & fire (via the toolkit)

Use the toolkit scripts in
`Mcity-data-recording-toolkit/trigger_mode/`:

```bash
# 1) launch all six cameras in action_trigger_mode (cam1 is the action master)
./lucid-drivers-action-trigger-mode.sh

# 2) wait for PTP to lock (PtpStatus -> Slave on all cameras), then fire one
#    synchronized shot across all six:
./trigger-all.sh

# optional: steady manual rate from the shell
./trigger-all-rate.sh 5      # ~5 Hz
```

For a hardware-precise continuous rate, prefer the in-driver timer:
launch with `action_trigger_rate:=<hz>` on the master (the toolkit launcher has a
commented line for this) instead of the shell loop.

## Parameters (new)

| Param | Type | Default | Meaning |
|---|---|---|---|
| `action_trigger_mode` | bool | `false` | Enable `TriggerSource=Action0` synchronized capture. |
| `action_master` | bool | `false` | This node broadcasts the action command and owns `/trigger_all`. Set on **exactly one** camera. |
| `action_device_key` | int | `1` | Action device key. **Identical on all six** + the fire command. |
| `action_group_key` | int | `1` | Action group key. **Identical on all six** + the fire command. |
| `action_group_mask` | int | `1` | Action group mask. **Identical on all six** + the fire command. |
| `action_lead_time` | double (s) | `0.05` | Margin added to the execute time to clear worst-case network latency/jitter. |
| `action_trigger_rate` | double (Hz) | `0.0` | `>0` on the master enables continuous triggering (driver timer). `0` = one-shot only. |
| `action_get_image_timeout` | int (ms) | `1000` | GetImage timeout in the action acquisition loop (timeouts are swallowed). |
| `ptp_domain` | int | `0` | PTP domain. Applied only when non-zero (best effort; see caveats). |
| `ptp_lock_timeout` | double (s) | `60.0` | Max wait for `PtpStatus == Slave` before refusing to fire. |

Existing params (`serial`, `pixelformat`, `exposure_time`, `gamma`,
`target_brightness`, `frame_rate`, `topic`, `camera_name`, QoS…) behave as in the
stock package. In action mode a **fixed `exposure_time`** is recommended over
autoexposure for deterministic synchronized capture.

## ON-VEHICLE VERIFICATION CHECKLIST

Run these after building and launching. Nothing here was tested on the authoring
machine.

1. **Build:** `colcon build --packages-select arena_camera_node_trigger` succeeds.
2. **Service present:** `ros2 service list | grep /trigger_all` shows `/trigger_all`
   (exactly once — only the master registers it).
3. **PTP slave + grandmaster identity:** each camera's `PtpStatus` reads `Slave`
   and the grandmaster identity equals the OXTS RTK. The master node logs
   `PTP locked … 'Slave'` before it will fire. (Check per-camera node logs and,
   if available, the camera web UI / `arena` tools for grandmaster identity.)
4. **One shot → one frame each:** with all six idle, a single
   `ros2 service call /trigger_all std_srvs/srv/Trigger` yields **exactly one**
   new frame on each `/arenacamN/images`, and the six `header.stamp` values are
   near-identical (sub-millisecond to low-millisecond apart, all in RTK GPS time).
5. **Continuous rate:** with `action_trigger_rate:=R` (or the shell-loop variant),
   `ros2 topic hz /arenacamN/images` reports ≈ `R` on every camera.
6. **No log spam while idle:** between triggers, nodes do **not** spam GetImage
   timeout warnings (timeouts are swallowed in action mode).

## CAVEATS to confirm on the vehicle

- **PTP DOMAIN must match** across RTK grandmaster, switch, and cameras. LUCID
  cameras commonly assume **domain 0**. If your grandmaster uses a non-zero
  domain, set `ptp_domain:=N` (best effort — the camera may not expose a writable
  domain node; verify the grandmaster/switch/camera are all on the same domain).
- **Six simultaneous bursts on one NIC** can overrun buffers. If you see dropped
  packets/incomplete frames, tune per-camera packet delay / frame-transmission
  delay (`GevSCPD` / `GevSCFTD`) to stagger transmission, and confirm jumbo
  frames + `StreamAutoNegotiatePacketSize` (already enabled). `action_lead_time`
  must exceed worst-case scheduling jitter.
- **Timestamps are RTK GPS time**, not UTC/Unix. Image `header.stamp` is derived
  from the camera PTP clock = RTK GPS timebase. Apply the GPS↔UTC offset (leap
  seconds) downstream for fusion. Note this in your data pipeline.
- See `ASSUMPTIONS-AND-VERIFY.md` for SDK node-name `[verify]` items.
