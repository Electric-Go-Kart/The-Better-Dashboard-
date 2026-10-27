# ML Integration Spec: Replacing CARLA Ground Truth on the Raspberry Pi

Status: Draft v1. Supersedes sections 6.2 and 6.1 (core layout) of
[what-the-kart-sees-spec.md](./what-the-kart-sees-spec.md). The BEV UI, data contract
(section 4 of that spec) and CARLA bridge stay as they are.

**Target:** Raspberry Pi 5 (BCM2712, 4x Cortex-A76 @ 2.4 GHz), Boot2Qt (Yocto), native C++ / Qt 6 app.
**Use case:** outreach demos, daytime, outdoors. Advisory display now; speed limiting or cut-off possible later (section 11).

---

## 1. Goal and scope

Replace the CARLA ground-truth object feed with a real perception pipeline:

| Input | Hardware |
|---|---|
| 2 x USB cameras | Logitech HD 1080p, 30 fps, 78° diagonal (70.4° H x 43.3° V), autofocus disabled |
| 2D lidar | DFRobot SEN0589 / LDROBOT STL-27L, 0.03-25 m, ~10 Hz, single plane, 360° |

| Output | Consumer |
|---|---|
| Objects (class, x, y, d, velocity, id, src) | `DetectionModel` via the existing data contract |
| Drivable-area and lane polygons (new) | New BEV layer in `WhatTheKartSeesPage.qml` |
| Health/status (new) | Banner ("No sensor data", "ML degraded") and future safety layer |

Out of scope for v1: 3D lidar, accelerator (Hailo etc.; see 12), motor control.

## 2. Key decisions (from requirements review)

| Topic | Decision |
|---|---|
| Detector | YOLO-FastestV2 (NCNN). Fallback: NanoDet-Plus (NCNN) if mAP gate in section 9 fails. |
| Segmentation | TwinLiteNet Nano, INT8 (QAT), NCNN, drivable area + lane lines |
| Runtime | Tencent NCNN only, static link. ONNX Runtime, PyTorch Mobile and TFLite are banned. |
| Cameras | Two, one inference stream shared, alternating frames |
| Fusion location | Inside the Qt app, in a worker `QThread` |
| Seg output | New BEV layer (ground-plane polygons) |
| Classes | `person`, `vehicle`, `bike`, `pole/barrier`, `unknown` (no `cone`) |
| Training | Pretrained first; fine-tune later (Appendix A) |
| CARLA | Kept as a test source, selected by env var |
| Calibration | Required, procedure in section 7 |
| Rates | See section 5 |

## 3. Core and thread allocation

Thread affinity via `pthread_setaffinity_np`, enabled by `DASH_PIN_CORES=1`. Optional kernel `isolcpus=1,2,3`.

| Core | Workloads | Scheduling |
|---|---|---|
| 0 | Qt GUI thread and render thread only | default, nice -5 |
| 1, 2 | TwinLiteNet (NCNN, 2 threads), seg post-processing | SCHED_OTHER |
| 3 | STL-27L serial reader (QtSerialPort), YOLO-FastestV2 (its on-demand MJPEG decode runs inside this thread, see 4.1), fusion/tracker. Camera capture is only a cheap V4L2 buffer dequeue, no decode. | lidar reader SCHED_FIFO (low prio, e.g. 10); everything else SCHED_OTHER |

Notes and risks:
- Core 3 is oversubscribed relative to the baseline. The lidar load is small (921600 baud is at most ~92 kB/s, 47-byte packets), but YOLO (including its on-demand decode) and fusion share it. **Gate:** lidar packet loss 0 and YOLO within rate in section 5 under full load. If it fails, move fusion to core 0 only after measuring GUI frame time, or move YOLO to share cores 1-2 at lower priority.
- NCNN threads: `ncnn::set_cpu_powersave(0)`, `Option::num_threads` = 2 for TwinLiteNet, 1 for YOLO. Never let NCNN's OpenMP pool spawn beyond the pinned set. Pin each worker thread, not the process.
- Each worker sets its own affinity at thread start. `QThread` objects for parsing, YOLO, seg and fusion are separate so a slow stage never blocks another.
- The lidar parser keeps the real-time priority only for reading and CRC; it hands full revolutions to fusion through a latest-value mailbox.

## 4. Pipeline architecture

```
 USB cam L ─┐                          ┌─> YOLO-FastestV2 (core 3) ──┐
            ├─> capture (no decode)    ┤  decode on demand           │
 USB cam R ─┘     (core 3, alternating)└─> TwinLiteNet (cores 1-2) ──┼─> Fusion + tracker (core 3)
                                                                     │        │
 STL-27L ──> serial parser (core 3, FIFO) ─> revolution mailbox ─────┘        v
                                                          DetectionModel / SegLayerModel (core 0, QML)
```

### 4.1 Camera capture
- V4L2, MJPEG, 1280x720 request (or 640x360 if decode load requires). Autofocus **off**, focus fixed at infinity-ish (`v4l2-ctl --set-ctrl=focus_automatic_continuous=0,focus_absolute=<val>`); also lock exposure/white balance if the driver allows, to keep frames consistent.
- **MJPEG decode policy (decode only what a network will consume):**
  - The capture thread only dequeues V4L2 buffers and keeps the newest *compressed* JPEG per camera. It never decodes, and never re-encodes or sends anything.
  - Decode happens only when YOLO (or TwinLiteNet) is ready for a frame and the throttle (`DASH_YOLO_FPS`) allows it. At 5-10 FPS aggregate that is 5-10 decodes per second, not 60. Frames the throttle skips are never decoded.
  - Decode runs inside the consuming inference thread, so it cannot add a separate competing thread on core 3. TwinLiteNet and YOLO share one decoded image when they use the same frame (reference-counted, read-only).
  - Use `libjpeg-turbo` with DCT scaling (`scale_denom`) to decode straight to 640x360, which is far cheaper than full 720p and is enough for the network inputs. Full-size decode only for calibration or recording.
  - The old debug camera path (JPEG frames over UDP 5600 / `CameraFeed`) is **off** in `ml` mode. A debug view or recording (`DASH_RECORD`, `DASH_CAMERA_DEBUG=1`) writes the already compressed JPEG as is (no decode, no re-encode) from a lowest-priority thread (`SCHED_IDLE`) that drops frames when busy, and is never enabled for demos.
  - **Gate:** with the YOLO+fusion+lidar load, decode share of core 3 is logged. If it exceeds ~15% of the core, lower decode size or `DASH_YOLO_FPS` before moving anything else.
- Two cameras on separate USB controllers/ports where possible. Pi 5 has two USB 3.0 and two USB 2.0 ports; MJPEG at 720p fits USB 2.0, but put them on separate ports for bandwidth.
- Capture threads drop old frames; only the newest frame per camera is kept.
- Each frame is stamped with a monotonic capture time on arrival. (UVC does not give hardware timestamps reliably, so estimate and subtract a measured fixed latency constant.)
- **FOV note:** two 70.4° cameras give at most 140.8° total. The BEV currently draws 160°. Either mount them toed out for 140° with small overlap or less, and set `cameraFovDeg` in `KartGeometry.js` to the measured value. The spec sets the default to **140**. Anything outside it stays `unknown/lidar`.

### 4.2 Scheduling of inference
- Alternating: L, R, L, R. Each camera frame goes to YOLO; the same frame also goes to TwinLiteNet if its previous run has finished (seg is allowed to skip frames).
- Per-camera effective rate is half the aggregate rate (section 5).
- A frame older than 100 ms at dispatch time is dropped.

### 4.3 YOLO-FastestV2
- Input ~352x352 (letterbox), NCNN `.param/.bin`, FP16 storage off unless benchmarked faster on A76, NEON on.
- Pretrained COCO weights for v1, mapped to the six classes:

| COCO | Class |
|---|---|
| person | `person` |
| car, truck, bus | `vehicle` |
| bicycle, motorcycle | `bike` |
| stop sign, parking meter, fire hydrant | `pole/barrier` |

- NMS in-app, conf threshold 0.35 default, tunable (`DASH_YOLO_CONF`).
- Output: 2D boxes with class and confidence per camera.

### 4.4 TwinLiteNet Nano (INT8, QAT)
- **Model:** TwinLiteNet(Plus) **Nano**, trained with quantization-aware training (QAT), deployed as **INT8** in NCNN. This is the primary model, not a fallback.
- Input ~360x640 (tunable down to 288x512), two output heads: drivable area, lane lines.
- **Export path (offline only; ONNX never ships on the Pi):** QAT PyTorch model -> fold fake-quant into weights/scales -> ONNX/PNNX -> NCNN FP32 `.param/.bin` -> `ncnn2int8` with a calibration table. **Risk:** `onnx2ncnn`/PNNX do not consume QDQ nodes directly, and NCNN INT8 uses its own per-layer scale table. Plan: export the QAT weights, then build the NCNN table from the QAT-learned activation scales (or re-run NCNN's calibration on a QAT model, which is robust to it). Early spike required; compare INT8 NCNN output to the PyTorch fake-quant reference.
- NCNN INT8 only quantizes convolution/inner-product layers. Resize/upsample, sigmoid/argmax and concat layers run in float with dequantize/requantize between. Check the decoder/upsampling head for float fallbacks that eat the speed-up, and keep the final upsample at network resolution, doing the full-size resize after thresholding.
- The A76 has the Armv8.2 dot-product instructions, which NCNN's INT8 kernels can use. Confirm the Yocto NCNN build enables them (`-march=armv8.2-a+dotprod`) and benchmark with `benchncnn`.
- Post-processing: threshold, morphological cleanup, contour extraction, then project to the ground plane (section 6.3) and simplify polygons (Douglas-Peucker) to at most ~64 vertices each.
- Run on cores 1-2, 2 NCNN threads, `lightmode` on.
- If the rate gate is missed, fall back in this order: lower input size (e.g. 288x512), skip every other frame, FP16 Nano (to rule out an INT8 float-fallback problem).

#### Memory and cache
The Pi 5 has per-core L1 (64 KB) and L2 (512 KB) and only a shared 2 MB L3. There is no shared L2 between cores 1 and 2 and no cache partitioning, so sharing is managed by keeping the working set small:
- INT8 activations are 4x smaller than FP32, so the Nano network's working set largely fits L2/L3 and is much less likely to evict YOLO, lidar and UI data than the FP32 model. Verify; do not assume.
- Decode once at the seg input size into a read-only planar buffer (~0.7 MB). YOLO letterboxes from that same buffer. No second decode, no full-size copy.
- Preallocate a pool of 2-3 frame buffers and per-thread ncnn allocators. No per-frame `malloc`.
- Align inter-thread mailboxes and counters to 64 bytes (`alignas(64)`) to avoid false sharing.
- **Gate:** `perf stat -e cache-misses,LLC-load-misses` per thread with seg on and off. Lidar loss and UI frame time must not change with seg enabled.

### 4.5 Fusion and tracking (Qt worker thread)
Runs on each new lidar revolution (~10 Hz) and each new detection batch.

1. Range mask at 20 m on the raw lidar. No ground filtering: the STL-27L is single-plane, so ground and height removal from the old plan do **not** apply.
2. Kart self-hit mask (`Stl27lReader::addMaskDeg`, `KartGeometry.js` values).
3. For each 2D box, convert box left/right pixel columns to bearing angles using the calibrated camera model (section 7). Select lidar points inside that angular wedge (a 2D wedge instead of a 3D frustum, because the lidar is planar). Median range of those points gives `d`. Source `fused`.
4. If the wedge has no points or inconsistent ranges (spread above a threshold, e.g. two surfaces), estimate distance from box-bottom ground projection (homography). Source `camera`, `~` shown in UI.
5. Lidar points outside every wedge, plus all points outside the camera fan, are clustered (distance-scaled DBSCAN or simple adjacent-gap clustering on the polar scan). Min cluster size and persistence (3 of 5 scans). Clusters become `unknown`, source `lidar`.
6. Tracker: nearest-neighbour or Hungarian association in the kart frame, constant-velocity Kalman filter per object, stable `id`, keep a track about 500 ms after it vanishes. A class assigned by camera sticks to the track; when an `unknown` track later falls in a camera wedge and is classified it keeps its `id`.
7. Output velocities `vx`, `vy` from the Kalman state; hide arrows under 0.3 m/s (UI already does this).

### 4.6 Dash-side motion smoothing
Detections arrive at only ~5-10 Hz (YOLO) and ~10 Hz (lidar), but the BEV targets 20-30 Hz.
- The tracker publishes position and velocity with a timestamp. `DetectionModel` (or the fusion thread, at a fixed 25 Hz tick) **extrapolates** each object along its velocity between updates.
- On a new measurement, correct smoothly: blend the displayed position toward the measured one over ~100-150 ms instead of snapping. Large jumps (above e.g. 1.5 m) snap.
- Extrapolation is capped at ~300 ms after the last update; after that the object is flagged stale and fades.
- Extrapolated values are for display only; safety logic (section 11) must use measured values and their ages.

## 5. Performance targets

| Metric | Target | Notes |
|---|---|---|
| YOLO aggregate rate | 5-10 FPS | 2.5-5 FPS per camera (alternating) |
| TwinLiteNet rate | >= 8 FPS | May run on the newer of the two cameras |
| Lidar | 10 Hz revolutions, **0.00% packet loss** | CRC failures counted separately |
| BEV update | 25 Hz display tick | Extrapolated between measurements |
| Camera-to-display latency | < 150 ms (p95) | Measured by packet/frame timestamps |
| UI frame rate | 60 FPS constant | `QSG_RENDER_TIMING=1` |
| RAM (app + models) | < 750 MB | RSS measured after 10 min run |
| CPU temperature | < 80 °C sustained | Active cooler required; throttling flag logged |

The aggregate YOLO rate is a software throttle (a timer in the inference scheduler), tunable via `DASH_YOLO_FPS`.

## 6. Interfaces and data contract

### 6.1 Source selection
`DASH_DETECTION_SOURCE` = `ml` (default on Pi) | `carla` (existing UDP 5602 path) | `demo`.
`DetectionModel` receives the same `QList<Detection>` regardless of source. UDP 5602 JSON stays as is so the CARLA bridge keeps working, and the ML fusion thread may optionally also emit it to UDP for debugging (`DASH_DETECTION_UDP_ECHO=1`).

### 6.2 Object contract additions
Existing fields are unchanged (see section 4 of the BEV spec). Add optional fields:

| Field | Meaning |
|---|---|
| `t` | Measurement monotonic timestamp (ms) |
| `age` | Ms since last measurement (stale if above ~300) |
| `cam` | `0`, `1` or `-1` (source camera) |
| `conf` | Already present, camera confidence; for lidar-only objects, track confidence |

### 6.3 Segmentation contract (new)
New message and in-process type, ground-plane polygons in the lidar frame (x forward, y right, metres), same as objects:

```json
{"seg": {"t": 123456,
  "drivable": [[[x,y],[x,y],...]],
  "lanes":    [[[x,y],[x,y],...]]}}
```

- Drivable area: list of polygons (outer ring only, v1). Lanes: list of polylines.
- Clipped to the 20 m BEV range. Anything above the horizon or beyond homography validity (e.g. > 20 m, or near-horizon rows) is dropped.
- Receiver: new `SegLayerModel` (C++), drawn by a new QML layer behind the icons and above the lidar dots. Low-alpha green for drivable, white for lanes. Off by default if the kart runs without calibration.
- Also publishes a boolean `offRoad` (the kart footprint area ahead is not mostly drivable) for future warnings. Display only in v1.

### 6.4 Health message
Published at 1 Hz, and immediately on change:
`{"health": {"lidar": "ok|stale|error", "cam0": ..., "cam1": ..., "yolo_fps": 7.2, "seg_fps": 8.5, "latency_ms": 120, "temp_c": 62.0, "throttled": false}}`
The BEV shows "No sensor data" when all sensors are stale, closing the existing open item (detections never time out): objects clear after a **stale timeout of 1 s** without any update from fusion.

## 7. Calibration procedure

Needed once per camera mount, and repeated when a camera is moved.

1. **Intrinsics:** checkerboard (e.g. 9x6, known square size), 20+ views per camera at the capture resolution that will be used. OpenCV `calibrateCamera`. Store `fx, fy, cx, cy`, distortion. Do this on the desktop, not on the Pi. Re-do if resolution changes. Autofocus must stay off during and after calibration since focus changes the focal length.
2. **Camera to lidar extrinsics (yaw, position, height, pitch):** place 3-4 tall narrow targets (poles with reflective wrap) at known distances 3, 6, 10 m at different bearings in the camera fan. Record lidar bearing/range and the camera pixel column. Solve for yaw offset and lateral/longitudinal offset by least squares. Check a residual below ~1.5° bearing error.
3. **Ground homography:** put markers on flat ground at known (x, y) in the lidar frame (a grid, e.g. 2 m spacing out to 12 m). Mark their pixel positions; `findHomography`. Also verifies camera height and pitch.
4. **Store:** `calibration/camera_{0,1}.yaml` (or JSON) next to the existing calibration install rules (6.4 of the BEV spec). Loaded at startup, validated (reprojection error limits), refuses to enable seg layer and `camera` source distances if invalid, with a banner message.
5. **Validation:** a known pole or person at 5 m and 10 m must land within 0.5 m laterally and 1 m in range for homography-derived (`camera` source) distances; within 0.2 m for `fused`.

Because the kart is fixed-mount, flat-ground assumption holds within a track. Slopes or kart pitch (braking, bumps) will bias seg-derived ground distances; accepted for v1, noted as a limitation.

## 8. Build and deployment (Boot2Qt)

- Build NCNN from source with the Yocto toolchain, static, `NCNN_VULKAN=OFF`, `NCNN_OPENMP=ON` (or `NCNN_SIMPLEOCV=ON` to avoid OpenCV in the app), NEON on, install `libncnn.a` into the Qt kit sysroot; link into the app CMake. A Yocto recipe (`ncnn_git.bb`) or a prebuilt toolchain artifact; decide which at implementation.
- OpenCV is **not** required on the Pi. Image processing (resize, letterbox, contour, homography) uses small custom code or NCNN's `mat` helpers, and calibration tooling stays on the desktop. If contour extraction proves too heavy to hand-write, a minimal OpenCV (core + imgproc, static) is an accepted exception; the memory budget must be re-measured.
- MJPEG decode: `libjpeg-turbo` with scaled decode (see 4.1). Do not use `QImage` for the hot path (full-size decode, extra copies).
- Image needs: Qt SerialPort, Qt Network, `v4l-utils`, USB UVC kernel module (`uvcvideo`), `can-utils`/`libsocketcan` (existing).
- Models and calibration installed under `/opt/dashboard/models` and `/opt/dashboard/calibration` via CMake install rules. Paths overridable: `DASH_MODEL_DIR`, `DASH_CALIB_DIR`.
- Systemd unit alongside `deploy/systemd/dash-can0.service`, with env: `DASH_PIN_CORES=1`, `DASH_DETECTION_SOURCE=ml`, `DASH_LIDAR_SOURCE=serial`, camera device paths (`DASH_CAM0_DEV=/dev/video0`, `DASH_CAM1_DEV=/dev/video2`). Prefer udev symlinks (`/dev/kart_cam_left`, `/dev/kart_cam_right`) to survive re-enumeration.
- Models are versioned, with a checksum checked at startup.

## 9. Verification and acceptance gates

### 9.1 Offline model gates (desktop, before deploying)
| Gate | Pass |
|---|---|
| NCNN output matches the PyTorch/ONNX reference | max abs diff within tolerance on 50 frames |
| YOLO-FastestV2 on a kart test set (daytime, outdoor) | person recall >= 0.85 at 10 m, vehicle >= 0.8 (v1 pretrained: person/vehicle only gated) |
| If person recall < 0.8 at 10 m | switch to NanoDet-Plus and re-gate |
| TwinLiteNet drivable IoU on kart test set | >= 0.80 (pretrained BDD100K may fail on a closed course; fine-tune per Appendix A) |

### 9.2 On-device gates
| Gate | Method | Pass |
|---|---|---|
| Inference rates | log per-stage fps for 10 min | section 5 |
| Lidar loss | sequence/CRC counters, 30 min with all load | 0 dropped |
| UI | `QSG_RENDER_TIMING=1` | 60 FPS, no frames over 20 ms |
| Latency | LED/marker moved in front of camera, high-speed capture, or timestamp probe | p95 < 150 ms |
| Core placement | `ps -L -o tid,psr,comm -p <pid>` and `top -H` | threads on assigned cores |
| RAM | `/proc/<pid>/status` VmRSS | < 750 MB |
| Thermal | 30 min run in sun-equivalent ambient | no throttling flag |
| Stale handling | unplug lidar, each camera in turn | health reports, objects cleared in 1 s, UI still 60 FPS |
| Fusion accuracy | poles/people at marked distances | calibration validation limits in section 7 |
| Soak | 2 h run | no leaks (RSS drift < 5%), no crashes |

### 9.3 Regression via CARLA
`DASH_DETECTION_SOURCE=carla` stays working. A recorded-data replay mode is recommended (record camera JPEG, lidar scans and timestamps to disk on the kart, replay into the pipeline on desktop or Pi) so ML regressions can be compared to the ground truth from CARLA scenarios. Compare tracks and distance error per scenario.

## 10. Failure behaviour

| Failure | Behaviour |
|---|---|
| One camera lost | Remaining camera continues; that side's fan marked "no camera"; its objects become `unknown/lidar` |
| Both cameras lost | Lidar-only unknown objects, banner "Camera offline", seg layer hidden |
| Lidar lost | Camera objects shown with `~` distances (homography); banner "Lidar offline"; health flags `degraded` |
| All stale | Clear objects after 1 s, "No sensor data" |
| Inference overrun | Drop frames, never queue; log |
| Model load / calibration invalid | Refuse stage, run remaining stages, show banner |
| NCNN crash | Run the ML pipeline guarded by a watchdog; if feasible isolate in a child process later (Section 12) |

## 11. Safety (future speed limiting / cut-off)

v1 is advisory. Because later versions may act on throttle or cut-off, design now for:
- **ML is never the only protection.** Any automatic speed limiting or cut-off must sit behind hardware/firmware limits on the VESC and a physical e-stop that do not depend on the Pi.
- Measured (not extrapolated) values and their timestamps are what safety logic uses. Each object carries `age`.
- A heartbeat/watchdog: if the fusion thread has not published in a set time, the safety layer must treat it as sensor failure (the safe response chosen later), not "no obstacles".
- The 2D lidar sees one plane at ~0.4 m only. It misses low or high obstacles and cannot be relied on for obstacle-free claims. Document this limit for any future action logic.
- False-negative rate must be measured and specified before any control integration (separate safety spec).
- Control integration path (CAN to VESC) is **out of scope** here and requires its own review.

## 12. Risks and open items

| Risk | Mitigation |
|---|---|
| TwinLiteNet Nano INT8 misses 8 FPS on two A76 cores | Smaller input, frame skipping, check float fallbacks and dotprod build |
| QAT scales do not carry into NCNN INT8 (no QDQ support) | Early spike; build NCNN scale table from QAT scales or re-calibrate; compare against fake-quant reference |
| NCNN conversion fails for some TwinLiteNet ops | Rewrite ops before export; use PNNX; early spike |
| Core 3 overload | Profiling gate in section 3; fallback core moves |
| Pretrained models weak on the low kart camera view | Appendix A fine-tune |
| 140° combined FOV vs 160° UI | Set `cameraFovDeg` to measured; update BEV spec |
| UVC autofocus/exposure drift | Disable autofocus; lock exposure where possible; recalibrate if focus changes |
| USB bandwidth / re-enumeration | Separate ports, udev symlinks, MJPEG, reconnect logic |
| Camera timestamps imprecise | Fixed latency constant measured per setup |
| Seg projection error on slopes/pitch | Accept in v1; future IMU pitch correction |
| Sunlight glare, shadows | Test on the real course; exposure lock; daytime-only claim |
| Accelerator (Hailo etc.) | Unlikely given Yocto/Boot2Qt integration cost. Not planned. Revisit only if CPU gates fail repeatedly. |
| UDP loopback bind | Still applies to CARLA mode only; add `DASH_SENSOR_BIND` (BEV spec 6.5) |
| Timestamps/sequence numbers missing in UDP feeds | Add `t` and `seq` (section 6.2) |
| Distances are from the lidar, not the bumper | Unchanged |

Open questions still to settle during implementation: Pi image NCNN recipe approach, exact camera mounting angles and heights, whether record/replay lands in v1, whether the `SegLayerModel` is drawn by `Canvas` or `QSGGeometryNode`, and the lighting/weather boundaries for outreach days.

---

## Appendix A: Training and data plan (fine-tune later)

1. **v1:** pretrained COCO YOLO-FastestV2 (person, vehicle, bike), pretrained BDD100K TwinLiteNet Nano (INT8 QAT; if no pretrained QAT checkpoint exists, QAT fine-tune from the FP32 weights on BDD100K first).
2. **Data collection:** record synchronized camera JPEG (both cameras, ~2 fps), lidar scans and timestamps from the kart on the real course, daytime, varied sun angle, with people, vehicles, bikes, barriers. Target 2-5k labeled frames for detection, 1-2k for segmentation to start.
3. **Labeling:** CVAT or Label Studio. Detection boxes for the six classes (`unknown` is never labelled). Drivable area and lane masks for seg.
4. **Fine-tune:** desktop GPU, start from the pretrained weights, with augmentation for brightness, blur and a low camera height. Hold out a test set from a different day/location.
5. **Export:** PyTorch -> ONNX/PNNX -> NCNN. TwinLiteNet Nano is fine-tuned with QAT and shipped INT8 (see 4.4); keep the QAT recipe and calibration set in the repo. YOLO-FastestV2 stays FP32/FP16 unless benchmarks justify INT8; benchmark each since NCNN INT8 does not always beat FP16 on small models.
6. **Versioning:** each model release records dataset version, metrics (section 9.1), checksum and the commit of the training code.
7. **Privacy:** outreach events may record the public. Decide on retention and face blurring before recording at events; recording is off by default (`DASH_RECORD=0`).

## Appendix B: Environment variables (new)

| Variable | Default | Meaning |
|---|---|---|
| `DASH_DETECTION_SOURCE` | `ml` on Pi, `carla` on desktop | `ml`, `carla`, `demo` |
| `DASH_PIN_CORES` | 0 | `1` enables thread affinity |
| `DASH_CAM0_DEV`, `DASH_CAM1_DEV` | `/dev/video0`, `/dev/video2` | Cameras (left, right) |
| `DASH_YOLO_FPS` | 8 | Aggregate YOLO throttle |
| `DASH_YOLO_CONF` | 0.35 | Detection threshold |
| `DASH_MODEL_DIR` | `/opt/dashboard/models` | NCNN `.param/.bin` |
| `DASH_CALIB_DIR` | `/opt/dashboard/calibration` | Camera calibration |
| `DASH_SEG_LAYER` | 1 | Show drivable/lane layer |
| `DASH_RECORD` | 0 | Record sensors for replay (stores compressed JPEG, SCHED_IDLE thread) |
| `DASH_CAMERA_DEBUG` | 0 | Send camera JPEGs to UDP 5600 for debug view (no decode, lowest priority) |
| `DASH_DETECTION_UDP_ECHO` | 0 | Echo ML objects to UDP 5602 |

Existing variables (lidar, ports, CAN) are unchanged.

## Appendix C: Differences from the baseline spec

- UI is alone on core 0 (baseline shared it with YOLO).
- YOLO moved to core 3 with the lidar reader; the lidar reader gets the only real-time priority.
- YOLO target lowered to 5-10 FPS aggregate (two cameras, alternating), from 10-15.
- 140° combined FOV from the real cameras replaces 160°.
- NanoDet-Plus added as a gated fallback.
- LiDAR is a 2D STL-27L (a scan, not a point cloud), so wedge-based fusion replaces frustum fusion and no ground filtering applies.
- Added: segmentation BEV layer, calibration, health/stale handling, motion smoothing, safety notes, training plan.
- "0.00% packet loss guaranteed by isolation" is changed to a measured gate, since core 3 is shared and isolation alone cannot guarantee it.
