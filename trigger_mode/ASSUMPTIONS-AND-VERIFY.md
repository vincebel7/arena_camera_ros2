# ASSUMPTIONS & [verify] list

Everything below could **not** be confirmed against an installed Arena SDK,
camera, or running ROS2 on the authoring machine. Confirm each on the vehicle.
Items are grouped; "source" = confirmed from the fork's own code; "[verify]" =
needs checking on the vehicle SDK/hardware.

## Confirmed from the fork source (no action needed, listed for traceability)
- `Arena::SetNodeValue<...>`, `Arena::GetNodeValue<...>`, `Arena::ExecuteNode`
  call style and `GenICam::gcstring` for enum-valued nodes — matches existing
  `set_nodes_*` methods.
- `m_pDevice->GetNodeMap()`, `m_pDevice->GetTLStreamNodeMap()`,
  `m_pDevice->GetImage(timeout)`, `RequeueBuffer`, `StartStream/StopStream` —
  used by the stock code.
- PTP enable path (`PtpEnable`, `PtpSlaveOnly`) — kept from `set_nodes_ptp_()`.
- `TriggerMode`, `TriggerSource`, `TriggerSelector`, `FrameStart` — used by the
  stock software-trigger path.
- gcstring → std::string assignment — the stock code does this for `PixelFormat`,
  so it is reused for `PtpStatus`.
- Stream settings `StreamAutoNegotiatePacketSize` / `StreamPacketResendEnable`
  kept as-is.

## [verify] — device nodemap (each camera)
- `ActionUnconditionalMode` (enum "On"/"Off"), `ActionSelector` (int),
  `ActionDeviceKey`, `ActionGroupKey`, `ActionGroupMask` (int) — standard GigE
  Vision / LUCID action-command node names. Confirm names + value types.
- `TriggerSource = "Action0"` — confirm the exact enum entry name on these models
  (TRI023S-CC / TRT023S-CC).
- `PtpDataSetLatch` (command) and `PtpDataSetLatchValue` (int64, ns) — given by
  the task spec; confirm present and that the value is nanoseconds in the PTP/RTK
  timebase.
- `PtpStatus` enum string `"Slave"` for the locked-slave state. Other states seen
  during settling: Listening / Uncalibrated / etc. There is no portable
  "offset from master" node assumed; "settled" is approximated by 5 consecutive
  `Slave` reads (tune `required_consecutive` in `wait_for_ptp_lock_()` if needed).
- `AcquisitionFrameRateEnable` writable while configuring trigger mode (set to
  `false`; wrapped in try/catch if read-only).
- `PtpDomainNumber` — **likely not present/writable** on all LUCID models. Applied
  only when `ptp_domain != 0`, in try/catch. If absent, configure the PTP domain
  on the grandmaster/switch and leave `ptp_domain:=0`.

## [verify] — system nodemap (`m_pSystem->GetTLSystemNodeMap()`, action master)
The six broadcast action-command node names (task spec flagged these):
- `ActionCommandDeviceKey`, `ActionCommandGroupKey`, `ActionCommandGroupMask`
  (int)
- `ActionCommandTargetIP` (int; `0xFFFFFFFF` = subnet broadcast)
- `ActionCommandExecuteTime` (int64 ns, PTP/RTK timebase)
- `ActionCommandFireCommand` (command)
Confirm exact spellings and that they live on the **system** TL nodemap.

## [verify] — exception types
- `GenICam::TimeoutException` thrown by `GetImage()` on timeout (used to swallow
  idle gaps). Standard in LUCID Arena C++ examples; if the build cannot find it,
  fall back to catching `GenICam::GenericException` and inspecting the message.

## [verify] — toolkit (serials / topics / exposure)
- Camera serial → topic/name mapping is copied verbatim from
  `drivers/lucid-drivers-autoexposure-trigger-mode.sh`:
  `254300057→arenacam1, …058→2, …053→3, …056→4, …055→5, …054→6`.
- Which physical camera is the **action master** (script uses cam1 = serial
  254300057). Any one camera works; pick per wiring preference.
- `exposure_time` default in the launcher is a **placeholder** (`4000` µs). Set a
  real value for the rig/scene; fixed exposure is recommended in triggered mode.
- Units: Arena `ExposureTime` is microseconds; `action_lead_time` is seconds;
  `ActionCommandExecuteTime` is nanoseconds (handled in code).

## Design assumptions (decisions, not unknowns)
- Action keys/mask default to `1/1/1` and are passed identically to all six
  cameras and to the fire command.
- One-shot execute time = ceil(latched PTP → next whole second) + `action_lead_time`.
- Continuous execute time = previous + `1/action_trigger_rate`, re-anchored to the
  next whole second if it falls behind real PTP time.
- The action master is itself one of the six cameras; the broadcast reaches it too,
  so it also captures the synchronized frame.
- `CMakeLists.txt` adds `find_package(Threads REQUIRED)` and links
  `Threads::Threads` to the `start` target so `std::thread`/`std::mutex` link
  reliably (rclcpp usually propagates pthread already; this makes it explicit).
- Single-threaded `rclcpp::spin` in `main.cpp` is kept; the action acquisition
  loop is a separate `std::thread`. Device **stream** access (acquisition thread)
  and device **nodemap** access (spin thread: PTP latch / status) do not overlap
  on the same subsystem; fires are serialized by `m_fire_mutex_`. If the SDK
  proves otherwise on the vehicle, switch `main.cpp` to a MultiThreadedExecutor
  and/or add a nodemap mutex.
