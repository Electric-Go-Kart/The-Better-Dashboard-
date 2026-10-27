# What The Kart Sees: Spec Sheet

Status: UI and CARLA ground-truth pipeline implemented. Real detector, fusion and
worker-thread pipeline are planned (see "Pi integration").

## 1. Purpose

A bird's-eye view (BEV) screen, opened from Settings, showing what is around the kart out to
20 m. It uses small 2D icons and signals, not camera video. Anything the cameras cannot
classify, including everything outside the camera view, still appears as an `unknown` icon with
an approximate distance.

Available at any speed (driver awareness), unlike Settings itself.

## 2. Design choices

| Topic | Decision |
|---|---|
| View | Kart-centric, forward up. Kart sits at 62% down the panel so the forward (camera) side gets room. Objects behind the kart clip off past about 12 m. |
| Range | 20 m hard limit |
| Zones | Red under 5 m, amber under 10 m, green beyond. Fixed values. |
| Alerts | Visual only: zone colour, pulsing ring under 5 m, "Closest" badge in the banner |
| Units | Follows the dashboard Metric/Imperial setting (metres or feet) |
| Camera region | 160° forward fan split by a dashed centre line into L CAM and R CAM. Fan edges run out from the kart. |
| Classes and icons | person, vehicle, cone, bike, pole/barrier, unknown. Drawn from QML shapes (no image files). |
| Unknowns | Shown as icons for everything, static or moving |
| Camera badge | Small camera mark on objects classified by the camera (`fused` or `camera`). Lidar-only unknowns have none. |
| Approximate distance | `~` prefix when the distance comes from the camera only |
| Motion | Arrow per object, pointing along its motion, longer when faster. Hidden under 0.3 m/s. |
| Update rate | Target 20 to 30 Hz. Bridge sends at 25 Hz. Icons animate over 40 ms. |
| Kart size | Adjustable, see section 3 |
| Camera stream | Still received by `CameraFeed` but not displayed. Kept for the detector and debugging. |
| Lidar dots | Dim raw lidar returns drawn behind the icons |

## 3. Tunables

All in [KartGeometry.js](../UI/Dashboard/dashboard/KartGeometry.js):

| Name | Default | Meaning |
|---|---|---|
| `kartLengthM` | 3.0 | Kart body length |
| `kartWidthM` | 1.5 | Kart body width |
| `lidarBehindCentreM` | 0.5 | Lidar sits this far behind the kart's centre |
| `rangeM` | 20 | Display range |
| `dangerM`, `cautionM` | 5, 10 | Zone limits |
| `cameraFovDeg` | 160 | Combined camera field of view |

All object positions are relative to the **lidar**, which is the drawing origin. The kart body is
drawn forward of that origin by `kartLength/2 - lidarBehindCentre`.

Distances are measured from the lidar to the object's nearest edge (CARLA ground truth) or
nearest cluster point (real pipeline), not from the kart's bumper.

Environment variables on the dash:

| Variable | Default | Meaning |
|---|---|---|
| `DASH_DETECTION_PORT` | 5602 | UDP port for detections |
| `DASH_LIDAR_PORT` | 5601 | UDP port for raw lidar scans |
| `DASH_CAMERA_PORT` | 5600 | UDP port for camera JPEG frames |
| `DASH_BEV_DEMO` | 0 | `1` shows moving fake objects without any bridge |

## 4. Data contract

One JSON datagram per tick to `127.0.0.1:5602`. x is metres forward of the lidar, y is metres
to the right.

```json
{"objects": [
  {"id": 12, "class": "person", "x": 7.0, "y": -2.0, "d": 7.3,
   "src": "fused", "vx": 0.4, "vy": 1.1, "conf": 0.9}
]}
```

| Field | Values |
|---|---|
| `class` | `person`, `vehicle`, `cone`, `bike`, `pole/barrier`, `unknown` |
| `src` | `fused` (camera class plus lidar distance), `camera` (class, distance estimated), `lidar` (unknown) |
| `d` | Distance in metres. Optional, computed from x and y if missing. |
| `vx`, `vy` | Velocity in m/s in the kart frame. Optional. |
| `id` | Stable per object. Keep it when an unknown becomes classified. |

Receiver: [detectionfeed.cpp](../Backend/detectionfeed.cpp). It keeps only the newest datagram
and updates [detectionmodel.cpp](../Backend/detectionmodel.cpp). When the set of IDs is
unchanged, rows are updated in place so QML delegates and animations are not recreated.

Page: [WhatTheKartSeesPage.qml](../UI/Dashboard/WhatTheKartSeesPage.qml). Icons:
[DetectionIcon.qml](../UI/Dashboard/DetectionIcon.qml).

## 5. CARLA test setup (desktop)

The bridge [carla_vesc_bridge.py](../tools/carla_vesc_bridge.py) now sends:

- CAN status frames (as before)
- Camera JPEG frames to 5600 and lidar scans to 5601
- Ground-truth objects to 5602 at 25 Hz

Ground truth rules:
- Actors within range (20 m, measured to the nearest edge) are reported.
- Inside the 160° camera fan: real class from the CARLA blueprint, `src: fused`.
- Outside it: `class: unknown`, `src: lidar`. This mimics the real behaviour where only the lidar sees them.
- Two-wheeled vehicles become `bike`, walkers `person`, cone props `cone`, barrier or pole props `pole/barrier`.

Bridge options: `--no-detections`, `--det-port`, `--det-hz`, `--det-range`, `--det-fov`.
`--mock` sends fake moving objects without CARLA.

Run order: `DASH_CAN_INTERFACE=vcan0` on the dash, then the bridge. Open Settings, then
What The Kart Sees.

Test scenarios to try:
1. Walk a pedestrian from behind the kart to in front. It should go unknown, then person at the fan edge, keeping the same ID.
2. Park a vehicle at 4 m: red colour, pulse, banner turns red.
3. Switch the dash to Imperial: labels and ring text turn to feet.
4. Stop the bridge: objects disappear when the feed stops (currently the last frame stays; see open items).

## 6. Pi integration plan (Boot2Qt, Raspberry Pi 5)

This part is not implemented yet.

### 6.1 Threading and cores
- Move UDP reading and parsing into a worker `QThread`, not the GUI thread.
- GUI thread pinned to core 0. Worker threads on cores 1 and 2 using `pthread_setaffinity_np`, behind an env flag such as `DASH_PIN_CORES=1`.
- Workers send only small value types (`QList<Detection>`) through queued signals, at 20 to 30 Hz, dropping old data rather than queuing it. Register the types with `qRegisterMetaType`.
- Optional `isolcpus=1,2` on the kernel command line.

### 6.2 Lightest-math pipeline (from the architecture plan)
1. Range mask at 20 m, applied first to the raw lidar.
2. Stride or downsample points closer than 5 m.
3. Quantised 2D detector (NanoDet-Plus or YOLOv8-Nano INT8) on the camera frames, on its own thread at about 10 Hz.
4. Turn each 2D box into a 3D frustum and pick lidar points inside it with vector math. The median distance becomes `d`. Source is `fused`.
5. Lidar points outside every frustum, and outside the camera fan, are clustered into `unknown` objects. The full 360° is kept, so the 160° FOV cull from the original plan is removed.
6. A tracker assigns stable IDs, smooths positions and keeps objects about 500 ms after they vanish.

Filtering for unknowns: ground and height removal, kart self-hit mask (using `KartGeometry.js`), minimum cluster size, persistence (3 of 5 scans), distance-scaled cluster radius.

### 6.3 Rendering
- Pure Qt Quick, no QtWidgets or `QGraphicsView`.
- Current implementation uses `Canvas` for the backdrop and the lidar dots. If frame time on the Pi is too high, replace the lidar dots with a `QQuickItem` using `QSGGeometryNode`.
- Icons are a `Repeater` over `DetectionModel`. A few dozen objects is fine.

### 6.4 Boot2Qt image and deployment
- Use the `Boot2Qt_6_11_3_Raspberry_Pi...` kit. Qt Network and SerialBus must be in the image.
- Add the detector runtime, `libsocketcan` and `can-utils` to the image.
- A systemd unit for the app next to `deploy/systemd/dash-can0.service`, with `DASH_CAN_INTERFACE=can0`, the sensor ports and affinity settings.
- eglfs with hardware GL. Check with `QSG_INFO=1`.
- Add install rules for model and calibration files.

### 6.5 Testing the Pi against CARLA
- CARLA stays on the PC. Run the bridge with `--cam-host <pi-ip>`. The dash sockets currently bind to loopback only, so add a bind-address setting such as `DASH_SENSOR_BIND=0.0.0.0` first.
- Send CAN from the PC's `vcan0` to the Pi with `cannelloni`, or run a second bridge on the Pi.
- Measure per-core load (`top -H`), thread placement (`ps -L -o tid,psr,comm -p <pid>`), frame time (`QSG_RENDER_TIMING=1`) and camera-to-display latency using packet timestamps.

## 7. Open items

- Detections never time out: if the bridge stops, the last objects stay on screen. Add a stale timeout that clears them and shows "No sensor data".
- UDP binds to loopback only (see 6.5).
- Rear objects clip past about 12 m with the forward-biased layout.
- Distances are from the lidar, not the kart's edge.
- Ground-truth mode sees objects through walls, so the real lidar will be less complete.
- No packet timestamps or sequence numbers yet.

## Real lidar: DFRobot SEN0589 (LDROBOT STL-27L)

- 2D single-plane 360 deg DTOF, 0.03-25 m, about 21600 points/s at ~10 Hz. There is no height data, so no ground filtering is needed or possible.
- UART 921600 8N1 (per LDROBOT's `stl27l.launch.py`), 47-byte packets: `0x54 0x2C`, speed, start angle, 12 x (mm, intensity), end angle, timestamp, CRC8.
- Parsed by `Backend/stl27lreader.{h,cpp}` (QtSerialPort, CRC check, resync, revolution assembly) and pushed into `LidarFeed`, so the QML is unchanged.
- Enable with env vars: `DASH_LIDAR_SOURCE=serial`, `DASH_LIDAR_DEVICE` (default `/dev/ttyUSB0`), `DASH_LIDAR_BAUD` (921600), `DASH_LIDAR_YAW` (deg the sensor's 0 deg is rotated clockwise from kart forward), `DASH_LIDAR_MIRRORED=1` (upside-down mount). Default source stays the CARLA UDP feed.
- Sensor angles are assumed clockwise from above with 0 deg at the connector-side front; verify on the kart and adjust yaw/mirror.
- The CARLA bridge now simulates the same sensor (1 channel, 25 m, 21600 pts/s). Mount it low enough to hit obstacles (default 0.4 m).
- Pi: user needs `dialout` access to the tty, and the Boot2Qt image needs Qt SerialPort. Display range stays 20 m.
- Unverified on hardware: baud rate, angle direction/zero, and kart self-hit masking (`Stl27lReader::addMaskDeg`).
