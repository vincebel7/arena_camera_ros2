# Synchronized triggering (PTP + scheduled GigE Vision Action Commands)

`trigger_mode:=true` makes all six LUCID cameras capture on the **same instant**,
driven by a scheduled GigE Vision **Action Command** in the vehicle's RTK GPS
timebase. `trigger_mode:=false` is unchanged free-run.

> This change was authored on a machine with no ROS2 / Arena SDK / cameras, so it
> was **not built or tested here**. Build and verify on the vehicle using the
> checklist below. Items that could not be confirmed against an installed SDK are
> marked **[verify]**.

## Why

The OXTS AV200 RTK is the **PTP grandmaster**; the switch passes PTP through; the
cameras are PTP slaves (`PtpEnable=true`, `PtpSlaveOnly=true`). PTP alone only
synchronizes **timestamps**, not **captures**. Setting `TriggerSource=Action0`
and having one "action master" node broadcast a scheduled action command makes
every camera expose at the same scheduled time.

## What `trigger_mode=true` now does (changed)

Previously `trigger_mode=true` armed a **software** trigger (`TriggerSource=Software`,
fired one frame per `ros2 run ... trigger_image` call). That path was too slow and
unsynchronized. It is now **replaced** by action-command synchronized capture:

- `set_nodes_action_trigger_mode_()` configures: `TriggerSelector=FrameStart`,
  `TriggerMode=On`, `TriggerSource=Action0`, `ActionUnconditionalMode=On`,
  `ActionSelector=0`, `ActionDeviceKey/GroupKey/GroupMask` (from params). Disables
  free-run `AcquisitionFrameRateEnable` so the action command fully gates capture.
- Acquisition runs in a background `std::thread` (`publish_images_action_`) so the
  node still spins/serves services; idle GetImage timeouts between triggers are
  swallowed quietly.
- The **action master** (`action_master:=true`, exactly one node) owns the
  `/trigger_all` service and implements `fire_scheduled_action_command_()`:
  `PtpDataSetLatch` -> `PtpDataSetLatchValue` -> compute execute time -> set the
  `ActionCommand*` system nodes (broadcast `0xFFFFFFFF`) -> `ActionCommandFireCommand`.
- `wait_for_ptp_lock_()` refuses to fire until this node's `PtpStatus` settles on
  `Slave`.

`trigger_mode=false` behavior is unchanged (free-run, blocking publish loop).

### Backward compatibility note
The legacy `/<node>/trigger_image` software-trigger service and the
`trigger_image` client still exist but are **inert** under `trigger_mode=true`
(use `/trigger_all` instead). They can be removed later if desired.

## Build (ON THE VEHICLE)

Standard colcon build of this package (no new ROS dependencies; the build links
`Threads::Threads` for the acquisition thread):

```bash
cd <your colcon workspace>
colcon build --packages-select arena_camera_node
source install/setup.bash
```

## Launch & fire

Use the toolkit scripts in `Mcity-data-recording-toolkit/drivers/`:

```bash
./lucid-drivers-sync-trigger.sh   # 6 cameras, trigger_mode=true, cam1 = master
# ...wait for PTP lock (master logs: PTP locked ... 'Slave') ...
./trigger-all.sh                  # one synchronized shot
./trigger-all-rate.sh 5           # ~5 Hz shell loop (or set action_trigger_rate)
```

## Parameters

Existing params (`serial`, `pixelformat`, `exposure_time`, `gamma`,
`target_brightness`, `frame_rate`, `topic`, `camera_name`, QoS…) behave as before.
`trigger_mode` (existing) now selects synchronized action capture. New params:

| Param | Type | Default | Meaning |
|---|---|---|---|
| `action_master` | bool | `false` | This node broadcasts the action command and owns `/trigger_all`. Set on **exactly one** camera. |
| `action_device_key` | int | `1` | Action device key. **Identical on all six** + the fire command. |
| `action_group_key` | int | `1` | Action group key. **Identical on all six** + the fire command. |
| `action_group_mask` | int | `1` | Action group mask. **Identical on all six** + the fire command. |
| `action_lead_time` | double (s) | `0.05` | Margin added to the execute time to clear worst-case network latency/jitter. |
| `action_trigger_rate` | double (Hz) | `0.0` | `>0` on the master enables continuous triggering (driver timer). `0` = one-shot only. |
| `action_get_image_timeout` | int (ms) | `1000` | GetImage timeout in the acquisition loop (timeouts swallowed). |
| `ptp_domain` | int | `0` | PTP domain. Applied only when non-zero (best effort; see caveats). |
| `ptp_lock_timeout` | double (s) | `60.0` | Max wait for `PtpStatus == Slave` before refusing to fire. |

A **fixed `exposure_time`** is recommended over autoexposure in `trigger_mode`.

## ON-VEHICLE VERIFICATION CHECKLIST

1. **Build:** `colcon build --packages-select arena_camera_node` succeeds.
2. **Service present:** `ros2 service list | grep /trigger_all` shows `/trigger_all`
   exactly once (only the master registers it).
3. **PTP slave + grandmaster identity:** each camera's `PtpStatus` reads `Slave`
   and the grandmaster identity equals the OXTS RTK. The master logs
   `PTP locked … 'Slave'` before it will fire.
4. **One shot -> one frame each:** with all six idle, a single
   `ros2 service call /trigger_all std_srvs/srv/Trigger` yields exactly one new
   frame on each `/arenacamN/images`, and the six `header.stamp` values are
   near-identical (sub-ms to low-ms apart, all in RTK GPS time).
5. **Continuous rate:** with `action_trigger_rate:=R` (or the shell-loop variant),
   `ros2 topic hz /arenacamN/images` reports ≈ `R` on every camera.
6. **No log spam while idle:** between triggers, nodes do not spam GetImage
   timeout warnings.
7. **Regression:** `trigger_mode:=false` still free-runs exactly as before.

## CAVEATS to confirm on the vehicle

- **PTP DOMAIN must match** across RTK grandmaster, switch, and cameras. LUCID
  cameras commonly assume **domain 0**. If the grandmaster uses a non-zero domain,
  set `ptp_domain:=N` (best effort — the camera may not expose a writable domain
  node; confirm RTK/switch/camera all share the domain).
- **Six simultaneous bursts on one NIC** can overrun buffers. If you see dropped
  packets / incomplete frames, tune per-camera packet delay / frame-transmission
  delay (`GevSCPD` / `GevSCFTD`) to stagger transmission. `action_lead_time` must
  exceed worst-case scheduling jitter.
- **Timestamps are RTK GPS time**, not UTC/Unix. Image `header.stamp` derives from
  the camera PTP clock = RTK GPS timebase. Apply the GPS↔UTC (leap-second) offset
  downstream for fusion.

## ASSUMPTIONS & [verify] list

Confirmed from the existing fork source (no action needed): Arena `SetNodeValue`/
`GetNodeValue`/`ExecuteNode` call style; `GenICam::gcstring` for enum nodes;
`GetNodeMap()`, `GetTLStreamNodeMap()`, `GetImage/RequeueBuffer/Start/StopStream`;
PTP enable path; `TriggerMode/Source/Selector` + `FrameStart`; gcstring->std::string
assignment (used for `PixelFormat`, reused for `PtpStatus`); stream packet settings.

**[verify] — device nodemap (each camera):**
- `ActionUnconditionalMode` ("On"/"Off"), `ActionSelector`, `ActionDeviceKey`,
  `ActionGroupKey`, `ActionGroupMask`; `TriggerSource="Action0"` enum entry.
- `PtpDataSetLatch` (command) + `PtpDataSetLatchValue` (int64 ns).
- `PtpStatus` enum string `"Slave"`. "Settled" is approximated by 5 consecutive
  `Slave` reads (no portable offset node assumed).
- `AcquisitionFrameRateEnable` writable while configuring trigger (set false;
  try/catch if read-only).
- `PtpDomainNumber` — likely absent/read-only on some models; set only when
  `ptp_domain != 0`, in try/catch.

**[verify] — system nodemap (`m_pSystem->GetTLSystemNodeMap()`, action master):**
- `ActionCommandDeviceKey`, `ActionCommandGroupKey`, `ActionCommandGroupMask`,
  `ActionCommandTargetIP` (`0xFFFFFFFF` broadcast), `ActionCommandExecuteTime`
  (int64 ns), `ActionCommandFireCommand` (command).

**[verify] — exception type:** `GenICam::TimeoutException` from `GetImage()` on
timeout (standard LUCID pattern). If the build can't find it, fall back to
catching `GenICam::GenericException`.

**[verify] — toolkit:** serial->topic mapping
(`254300057→cam1, 058→2, 053→3, 056→4, 055→5, 054→6`); which camera is the master
(scripts use cam1); the `exposure_time=4000`µs placeholder.

**Design decisions (not unknowns):** keys/mask default `1/1/1`, identical on all
six and the fire command; one-shot execute = ceil(latched PTP -> next whole
second) + `action_lead_time`; continuous = previous + `1/action_trigger_rate`,
re-anchored to the next whole second if it falls behind PTP; the master is one of
the six cameras (the broadcast reaches it too); single-threaded `rclcpp::spin` is
kept and the acquisition loop is a separate `std::thread` (stream vs nodemap
access don't overlap on a subsystem; fires are serialized by `m_fire_mutex_`); a
`Threads::Threads` link was added to `CMakeLists.txt`.
