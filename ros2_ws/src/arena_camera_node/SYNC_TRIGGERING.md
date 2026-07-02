# Synchronized triggering — PTP + scheduled GigE Vision Action Commands

This document explains the changes made to the `arena_camera_node` driver to give
the six-camera LUCID rig **manual, PTP-synchronized triggering**: one command
fires a single frame from every camera at the *same instant*, all sharing the
OXTS RTK GPS timebase, with an optional steady triggered rate.

`trigger_mode:=true` now means synchronized action-command capture.
`trigger_mode:=false` is unchanged free-run.

> **Validated on the vehicle:** six cameras synchronized to a measured **6.71 µs**
> max cross-camera spread, and **30 Hz recording at ~99.9%** completeness
> (4 frames out of ~3925 over 2 min, most of which are start/stop boundary
> frames). Every `[verify]` SDK node name below was confirmed working.

---

## 1. Root cause — why triggering and sync did not work before

### 1a. `trigger_mode` used a *software* trigger
The stock `set_nodes_trigger_mode_()` set `TriggerSource = "Software"`. A software
trigger fires **one camera at a time** via a per-camera service call
(`/<node>/trigger_image` → `TriggerSoftware`). For six cameras that means six
independent calls from six processes, each landing at a different wall-clock
instant. There is no mechanism that makes the six sensors expose together — it is
inherently unsynchronized, and slow (one process per shot).

### 1b. PTP synchronized the *clocks*, not the *captures* — the key misconception
The driver already enabled PTP (`PtpEnable=true`, `PtpSlaveOnly=true`), and the
OXTS RTK is the PTP grandmaster. It is tempting to assume "PTP is on, therefore
the cameras are synchronized." **They are not.** PTP only disciplines each
camera's internal *clock* to the grandmaster, so their *timestamps* agree. It does
nothing to make them *expose at the same time*. With PTP + software trigger you
get accurate but **independent** capture instants that merely *carry* good
timestamps. Synchronized *capture* needs a separate mechanism that commands all
cameras to expose at one shared time.

That mechanism is **PTP + scheduled GigE Vision Action Commands**, which is the
gap this change fills.

---

## 2. What we implemented (driver)

All of this is gated behind the existing `trigger_mode` parameter; the
`trigger_mode:=false` free-run path is byte-for-byte unchanged.

1. **`set_nodes_action_trigger_mode_()`** — when `trigger_mode:=true`, configure
   each camera for action-command capture instead of software:
   `TriggerSelector=FrameStart`, `TriggerMode=On`, `TriggerSource=Action0`,
   `ActionUnconditionalMode=On`, `ActionSelector=0`, and the shared
   `ActionDeviceKey` / `ActionGroupKey` / `ActionGroupMask`. Free-run
   `AcquisitionFrameRateEnable` is disabled so the action command fully gates
   capture. PTP and the stream packet settings are kept.

2. **Threaded acquisition (`publish_images_action_`)** — in triggered mode frames
   arrive only when an action command fires, so the GetImage/publish loop runs in
   a `std::thread`. The node keeps spinning to serve services, and GetImage
   timeouts between triggers are swallowed quietly.

3. **Action master + `/trigger_all`** — exactly one node (`action_master:=true`)
   broadcasts the action command. It owns the absolute `std_srvs/srv/Trigger`
   service `/trigger_all` and implements `fire_scheduled_action_command_()`:
   - `PtpDataSetLatch` → read `PtpDataSetLatchValue` (this device's current PTP
     time, in the RTK GPS timebase, nanoseconds).
   - Compute an execute time in that timebase.
   - On `m_pSystem->GetTLSystemNodeMap()`: set `ActionCommandDeviceKey`,
     `ActionCommandGroupKey`, `ActionCommandGroupMask`,
     `ActionCommandTargetIP=0xFFFFFFFF` (subnet broadcast),
     `ActionCommandExecuteTime`, then `ActionCommandFireCommand`.
   The single broadcast reaches all six cameras (including the master's own).

4. **PTP lock gate (`wait_for_ptp_lock_`)** — the master refuses to fire until its
   own `PtpStatus` settles on `Slave` (sustained for several reads). Firing before
   lock would latch a meaningless time.

5. **Continuous rate** — `action_trigger_rate:=<Hz>` on the master drives a timer
   that fires repeatedly for a steady synchronized stream.

6. **Transmission staggering (`gev_scftd` / `gev_scpd`)** — see §4; per-camera
   `GevSCFTD` spreads *when each camera transmits* its frame to relieve the host
   at high rates, without touching capture timing.

### The continuous-scheduler bug we fixed (this was the "stuck at ~4.5 Hz" cause)
The first continuous implementation rounded the first trigger **up to the next
whole second** and then stepped from there, so every action command was scheduled
**~1 second in the future** — leaving ~10 commands "in flight" at once. **LUCID
cameras hold only one pending scheduled action**, so all but one were dropped,
throttling the achieved rate to ~3–4.5 Hz regardless of the requested rate.

The fix: in continuous mode, schedule each trigger just `action_lead_time` ahead
of a *freshly latched* PTP time, so **exactly one command is in flight** at a
time. The camera then fires at the timer rate (verified clean 10 Hz, and 30 Hz
with staggering).

---

## 3. Why it works

- **Common clock + scheduled command = simultaneous exposure.** PTP gives all six
  cameras one shared clock (the RTK grandmaster). A scheduled action command names
  a single absolute execute time on that clock; every camera, having the same
  clock, fires its `FrameStart` at that same instant. Result: simultaneous
  exposure, measured at **6.71 µs** max cross-camera spread (essentially just the
  residual inter-camera PTP offset).
- **One broadcast, identical keys.** The device/group key/mask are identical on
  all cameras and on the fire command, and `ActionCommandTargetIP` is the subnet
  broadcast, so a single packet triggers all six together — no per-camera round
  trips.
- **One pending action at a time.** Scheduling each continuous trigger only
  `action_lead_time` ahead matches the camera's single-pending-action model, so
  nothing is dropped from a backlog and the requested rate is actually achieved.
- **`header.stamp` is the capture time.** The driver sets the message stamp from
  `pImage->GetTimestampNs()` (the camera's FrameStart timestamp in PTP/RTK time),
  not the ROS clock — so the timestamp *is* the synchronized capture instant, and
  it is unaffected by anything downstream (e.g. transmission staggering).

---

## 4. Transmission staggering (`gev_scftd` / `gev_scpd`)

At high rates (30 Hz), synchronized capture has a side effect: all six frames hit
the network at the *same instant*, and those 6-wide bursts overflow the host's
receive buffers. `GevSCFTD` (stream-channel frame-transmission delay), set to a
different value per camera (cam1=0, cam2=Δ, … cam6=5Δ), staggers **when each
camera transmits** its already-captured frame so they arrive spread across the
window instead of all at once. `GevSCPD` (packet delay) can further throttle each
camera's burst.

This shapes **transmission only** — the trigger/exposure is still simultaneous, so
`header.stamp` and the ~7 µs sync are unchanged. Constraint:
`(5 × Δ) + one-frame-transmission-time < trigger period (1/rate)`, or the last
camera's buffer overruns. At 30 Hz (33 ms period) and ~6 ms/frame, Δ ≈ 4 ms works.

---

## 5. Parameters

Existing params (`serial`, `pixelformat`, `exposure_time`, `gamma`,
`target_brightness`, `frame_rate`, `topic`, `camera_name`, QoS…) behave as before.
`trigger_mode` (existing) now selects synchronized action capture. New params:

| Param | Type | Default | Meaning |
|---|---|---|---|
| `action_master` | bool | `false` | This node broadcasts the action command and owns `/trigger_all`. Set on **exactly one** camera. |
| `action_device_key` | int | `1` | Action device key. **Identical on all six** + the fire command. |
| `action_group_key` | int | `1` | Action group key. **Identical on all six** + the fire command. |
| `action_group_mask` | int | `1` | Action group mask. **Identical on all six** + the fire command. |
| `action_lead_time` | double (s) | `0.05` | Margin added to the execute time to clear worst-case latch/fire/network latency. In continuous mode this is the (only) lead, so keep it **below `1/rate`**. |
| `action_trigger_rate` | double (Hz) | `0.0` | `>0` on the master enables continuous triggering. `0` = one-shot only (fire via `/trigger_all`). |
| `action_get_image_timeout` | int (ms) | `1000` | GetImage timeout in the acquisition loop (timeouts swallowed). |
| `ptp_domain` | int | `0` | PTP domain. Applied only when non-zero (best effort). |
| `ptp_lock_timeout` | double (s) | `60.0` | Max wait for `PtpStatus == Slave` before refusing to fire. |
| `gev_scpd` | int | `0` | GigE stream-channel packet delay (transmission shaping). `0` = unchanged. |
| `gev_scftd` | int | `0` | GigE stream-channel frame-transmission delay (per-camera stagger). `0` = unchanged. |

> Note: `exposure_time` is declared as a **double** — pass it as a float
> (`4000.0`, not `4000`), or rclcpp rejects the integer override. A **fixed**
> exposure is recommended in `trigger_mode` for deterministic capture.

**Image flip:** `reverse_x` / `reverse_y` (bool, default `false`) toggle the
camera's `ReverseX` / `ReverseY` nodes — the same switches as in ArenaView (set
both `true` for a 180° flip). **Bayer caveat:** flipping a Bayer image changes the
effective Bayer pattern (e.g. `RGGB` → `GRBG` for a single-axis flip, `BGGR` for
both), but the published encoding string does not change — so debayered colors can
come out wrong. If that happens, either use `pixelformat:=rgb8` (the camera
debayers internally *before* the flip, so the pattern issue disappears) or set
`pixelformat` to the flipped Bayer order to match.

---

## 6. Build (on the vehicle)

`trigger_mode` synchronized capture is gated entirely by params; the only build
change is `Threads::Threads` linkage for the acquisition thread (added to
`CMakeLists.txt`). No new ROS dependencies.

```bash
cd <your colcon workspace>
rm -rf build install log          # clean rebuild picks up CMakeLists + source changes
colcon build --packages-select arena_camera_node
source install/setup.bash
```

---

## 7. On-vehicle verification checklist

1. `colcon build --packages-select arena_camera_node` succeeds.
2. `ros2 service list | grep /trigger_all` shows `/trigger_all` exactly once
   (only the master registers it).
3. Each camera's `PtpStatus` reads `Slave`, grandmaster identity = the OXTS RTK;
   the master logs `PTP locked … 'Slave'` before it fires.
4. One `ros2 service call /trigger_all std_srvs/srv/Trigger` yields exactly one new
   frame per camera with near-identical `header.stamp` (we measured 6.71 µs).
5. With `action_trigger_rate:=R`, `ros2 bag info` shows ~`R × duration` per camera,
   all six roughly equal. (Use `bag info` — `ros2 topic hz/bw` under-report when
   the publisher is reliable and under recording load.)
6. `trigger_mode:=false` still free-runs exactly as before.

---

## 8. Caveats — all confirmed working on the vehicle

The following SDK node names were marked `[verify]` during authoring (no SDK was
available) and have since been **confirmed correct** on the vehicle's Arena SDK /
TRI023S-CC / TRT023S-CC cameras:

- Device nodemap: `TriggerSource="Action0"`, `ActionUnconditionalMode`,
  `ActionSelector`, `ActionDeviceKey/GroupKey/GroupMask`, `PtpDataSetLatch`,
  `PtpDataSetLatchValue`, `PtpStatus=="Slave"`, `GevSCFTD`/`GevSCPD`.
- System TL nodemap: `ActionCommandDeviceKey/GroupKey/GroupMask/TargetIP/`
  `ExecuteTime`/`FireCommand`.
- `GenICam::TimeoutException` from `GetImage()`.

Still to keep in mind:
- **PTP domain** must match across RTK / switch / cameras (cameras assume domain
  0; we run domain 0). `ptp_domain` is best-effort if a camera lacks a writable
  domain node.
- **`gev_scftd` units** — treated as nanoseconds; if effective staggering looks
  off, confirm the unit on your SDK and re-tune the launcher's `scftd_step`.
- **Timestamps are RTK GPS time**, not UTC/Unix. Apply the GPS↔UTC leap-second
  offset downstream for fusion.

---

## 9. Change summary (debugging journey, for the next person)

| Symptom observed | Root cause | Fix |
|---|---|---|
| Cameras not actually synchronized under `trigger_mode` | Software trigger; PTP only synced clocks, not captures | `TriggerSource=Action0` + scheduled action commands |
| No frames flowing | `action_trigger_rate=0.0` (no auto-fire) | Set a rate, or fire `/trigger_all`; launcher now defaults to 10 Hz |
| Continuous rate stuck at ~4.5 Hz | Scheduler queued ~1 s of commands; camera holds only one pending | Schedule each trigger `action_lead_time` ahead (one in flight) |
| Node hangs on a rejected param | `exposure_time` passed as int (declared double) | Pass `4000.0` |
| Heavy frame loss at 30 Hz | Six synchronized frames burst the host at once | `GevSCFTD` per-camera transmission staggering (this driver); reliable QoS + buffers (recorder side) |

See the toolkit's `SYNC_RECORDING.md` for the recording-side analysis (QoS,
buffers, memory) and the launch/trigger/verification tooling.
