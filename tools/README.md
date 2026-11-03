# Running the dashboard with CARLA

CARLA drives the dashboard through a virtual CAN bus (`vcan0`). The bridge
([carla_vesc_bridge.py](./carla_vesc_bridge.py)) sends VESC-style status frames
for the left (ID 1) and right (ID 2) motors, and the dashboard reads them as if
they came from real VESC controllers.

## One-time setup

```bash
# Python env (CARLA wheel is cp312 for 0.9.16)
cd tools
uv venv --python 3.12 .venv-carla
source .venv-carla/bin/activate
uv pip install python-can pygame numpy \
  /media/joshua/SamsungSSD/carla-ue4/carla/PythonAPI/carla/dist/carla-0.9.16-cp312-cp312-linux_x86_64.whl

# Optional: CAN debugging tools
sudo apt install can-utils
```

## Every boot: virtual CAN bus

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

## Run (one terminal each)

**Terminal 1: CARLA server.** Wait for the map to finish loading.

```bash
cd /media/joshua/SamsungSSD/carla-ue4/carla/Dist/CARLA_Release
./CarlaUE4.sh -quality-level=Low -windowed -ResX=1280 -ResY=720
```

**Terminal 2: drive the car** (spawns the `hero` vehicle; drive with W/A/S/D).

```bash
source ~/Desktop/gokart/dashboard/The-Better-Dashboard-/tools/.venv-carla/bin/activate
python /media/joshua/SamsungSSD/carla-ue4/carla/PythonAPI/examples/manual_control.py
```

**Terminal 3: CARLA to CAN bridge.**

```bash
source ~/Desktop/gokart/dashboard/The-Better-Dashboard-/tools/.venv-carla/bin/activate
python ~/Desktop/gokart/dashboard/The-Better-Dashboard-/tools/carla_vesc_bridge.py --iface vcan0
```

**Dashboard:** run `appQMLCDash` from Qt Creator (Projects > Run > Environment):

```
DASH_CAN_INTERFACE=vcan0
```

The bridge also attaches a front camera to the `hero` vehicle and streams it (320x180
JPEG, 15 fps, UDP to `127.0.0.1:5600`) to Settings > What The Kart Sees. Needs
`pip install pillow numpy` in the venv. Change the port with `--cam-port` and
`DASH_CAMERA_PORT`; disable with `--no-camera`.

It also attaches a single-plane lidar modelled on the STL-27L (1 channel, 21600 points/s, 25 m) and
streams top-down scans to `127.0.0.1:5601` (`--lidar-port` / `DASH_LIDAR_PORT`, `--no-lidar` to skip).
These appear under the camera on the same page.

Detected objects (ground truth from CARLA: actors in the camera fan keep their class,
the rest are `unknown`) go to `127.0.0.1:5602` (`DASH_DETECTION_PORT`). See
[docs/what-the-kart-sees-spec.md](../docs/what-the-kart-sees-spec.md).
`DASH_BEV_DEMO=1` shows fake objects on the dash without the bridge.

Leave `DASH_CAN_TEST_MODE` and `DASH_UI_SWEEP_TEST` unset. Both replace the live data.

## Checks

- `candump vcan0` should show frames `00000901` and `00000902`.
- Run only one bridge at a time. A leftover `--mock` bridge sends a synthetic sweep
  on the same IDs and mixes with the real values. Check with
  `ps aux | grep carla_vesc_bridge | grep -v grep`.
- `--mock` runs the bridge without CARLA, which is useful for testing the dashboard alone.

## Bridge options

`python carla_vesc_bridge.py --help` lists them all. The main ones:

| Option | Default | Meaning |
|---|---|---|
| `--iface` | `vcan0` | CAN interface (`can0` on the Pi) |
| `--left-id`, `--right-id` | 1, 2 | Must match `DASH_LEFT_CONTROLLER_ID` / `DASH_RIGHT_CONTROLLER_ID` |
| `--wheel-radius` | 0.1207 m | Matches the speedometer's 9.5 in wheel |
| `--gear-ratio` | 30/9 | Matches the speedometer's gear teeth |
| `--max-current` | 60 A | Current at full throttle |
| `--cells`, `--capacity-ah` | 13, 20 | Battery model |
| `--role` | `hero` | CARLA vehicle `role_name` to follow |
| `--cam-width`, `--cam-height`, `--cam-fps`, `--cam-quality` | 320, 180, 15, 50 | Camera stream size and rate |
| `--cam-x`, `--cam-z` | 0.8, 1.5 m | Camera position on the vehicle |
| `--lidar-range`, `--lidar-pps`, `--lidar-hz` | 25 m, 21600, 10 | Lidar density and range |
| `--lidar-x`, `--lidar-z` | 0.3, 0.4 m | Lidar position on the vehicle |
| `--det-port`, `--det-hz`, `--det-range`, `--det-fov` | 5602, 25, 20 m, 160 | Ground-truth object stream to the BEV page |
| `--no-detections` | off | Skip the object stream |
| `--no-lidar` | off | Skip the lidar stream |
| `--no-camera` | off | Skip the camera stream |
| `--mock` | off | Synthetic drive cycle, no CARLA needed |

## ML camera harness (YOLO-FastestV2 + TwinLiteNet on CARLA frames)

[ml_camera_harness.py](./ml_camera_harness.py) is a standalone desktop check of YOLO-FastestV2 and
TwinLiteNet before they go into the dash (see [docs/ml-integration-spec.md](../docs/ml-integration-spec.md)).
It reads the bridge's camera JPEG stream (or a folder of images), runs either or both models, and shows boxes
(person, vehicle, bike, pole/barrier), the drivable area (green) and lane lines (red). It does not touch the dash.

Two backends:
- `--backend torch` (default): runs the `.pth` weights straight from the model repos. Use this first.
- `--backend ncnn`: runs converted `.param/.bin` files (the deployment format). Run it later on the same frames to compare against torch.

Model repos are looked up in `~/Desktop/gokart/models` (override with `ML_MODELS_DIR` or `--yolo-repo` / `--seg-repo`):
`yolo/Yolo-FastestV2` (weights `modelzoo/coco2017-0.241078ap-model.pth`, config `data/coco.data`) and
`twinlitenet/TwinLiteNet` (weights `pretrained/best.pth`). Override with `--yolo-pth`, `--yolo-data`, `--seg-pth`.

Use its own venv, separate from `.venv-carla` (Python 3.12 may not suit your ML packages):

```bash
python3 -m venv .venv-ml && source .venv-ml/bin/activate
pip install numpy opencv-python torch torchvision torchsummary   # torch backend (torchsummary is imported by the YOLO repo)
pip install ncnn                                                  # ncnn backend
```

Run (CARLA + bridge as above). The bridge default of 320x180 is too small for the networks. Use a larger frame
that still fits one UDP datagram (JPEG under 65000 bytes):

```bash
python carla_vesc_bridge.py --iface vcan0 --cam-width 640 --cam-height 360 --cam-fov 70.4 --cam-quality 70 --cam-fps 10
python ml_camera_harness.py --backend torch
# later, converted models:
python ml_camera_harness.py --backend ncnn \
    --yolo-param yolo-fastestv2.param --yolo-bin yolo-fastestv2.bin \
    --seg-param twinlitenet_nano_int8.param --seg-bin twinlitenet_nano_int8.bin
```

- Port 5600 is shared with the dash. Close the dash, or start the bridge with `--cam-port 5610` and the harness with `--port 5610`.
- `--no-yolo` / `--no-seg` skip a model. For `ncnn`, a model is skipped when its files are not given.
- `--folder DIR [--loop]` replays saved images (repeatable runs, torch vs NCNN comparison). `--save-dir DIR` writes annotated frames. `--no-display` is headless and only prints per-frame timings.
- Other options: `--device cuda` (torch), `--yolo-conf`, `--seg-width/--seg-height` (default 640x360), `--all-classes` (show every COCO class).
- NCNN blob names: `--yolo-in`, `--yolo-out` (comma separated, stride 16 then 32), `--seg-in`, `--seg-out-da`, `--seg-out-ll`. Defaults are guesses, so set them to match your converted model.
- Timings are desktop-relative only. Measure real speed on the Pi 5.

Notes and caveats:
- The torch YOLO path was checked against the repo's own `img/000139_result.png`: same boxes and scores. YOLO takes the BGR image as is (no colour swap), TwinLiteNet takes RGB, both scaled 0..1.
- The NCNN YOLO path expects a model exported with the repo's `pytorch2onnx.py` (sigmoid/softmax baked in, two outputs). **Untested**, since no NCNN model existed when this was written.
- The NCNN TwinLiteNet path assumes two outputs of 2-channel logits (class 1 = drivable / lane). **Untested.** The torch TwinLiteNet path runs but its masks have not been checked on road images.
- The torch run assumes the plain FP32 TwinLiteNet. A Nano or QAT checkpoint needs its own model class.
- CARLA's rendering is a poor stand-in for a real camera. Use it to catch plumbing errors (colour order, scaling, class mapping, mask orientation, INT8 vs FP32 drift), not to judge accuracy.

## Lidar harness (real STL-27L parser + lidar pipeline on CARLA scans)

[lidar_harness.py](./lidar_harness.py) drives the production lidar code from the CARLA bridge and shows what it detects.
It does not touch the dash. Data path:

```
CARLA scan (UDP 5601)  or  --synthetic scene
   -> accumulate to full 2160-ray revolution, add range noise, apply 0.03-25 m limits
   -> encode as real 47-byte STL-27L packets (CRC8)
   -> lidarHarnessCli (C++): Backend/stl27lreader.cpp (framing, CRC, angles, mount yaw/mirror, masks)
                             Backend/lidarpipeline.cpp (20 m mask, body mask, clustering, 3-of-5 persistence, Kalman tracker)
   -> JSON per revolution -> OpenCV top-down view + score against bridge ground truth (UDP 5602)
```

Build the C++ driver once (needs Qt Core + SerialPort only, not the whole dash):

```bash
cmake -S tools -B build/lidar_harness -G Ninja -DCMAKE_PREFIX_PATH=$HOME/Qt/6.x.y/gcc_64
cmake --build build/lidar_harness      # if the compiler fails with missing std headers, add -j1
```

Run (python env needs numpy and opencv-python, e.g. `.venv-ml`):

```bash
python lidar_harness.py --synthetic                          # no CARLA: moving person, cars, pole, cone, wall
python lidar_harness.py                                      # live: bridge running with its lidar + detection streams
python lidar_harness.py --synthetic --no-display --score     # headless, exit code 1 if thresholds fail
```

- Live mode binds UDP 5601 (scans) and 5602 (ground truth), shared with the dash. Close the dash or move the bridge
  to other ports (`--lidar-port`, `--det-port` on both sides).
- Kart body rectangle comes from `UI/Dashboard/dashboard/KartGeometry.js`. Sensor-angle self-hit wedges: `--mask FROM,TO` (repeatable).
- `--mount-yaw DEG` and `--mirrored` are applied in both the encoder and the C++ reader, so non-default values must give identical results.
- Sensor model: `--noise` (m, default 0.015), `--min-m`, `--max-m`, `--dropout`; `--corrupt P` drops/bit-flips a fraction of packets to exercise the CRC and resync path.
- Pipeline tuning without rebuilding: `--set gateM=2 --set persistHits=2` (see `LidarPipeline::Params`).
- Colours: green T = confirmed track (arrow = velocity above 0.3 m/s), red T = false positive, circle/box = ground truth (red = missed), blue box = cluster. Keys: `q` quit, space pause, `s` save PNG.
- Score: precision, recall, position error, velocity error, ID switches, detection latency. A track matches a ground-truth object within `--gate` plus the object's half-size. Objects are only "missed" after `--grace` scans.

Camera fusion (spec §4.5 steps 3-4):
- `--boxes gt|yolo|none`. `gt` projects ground truth into the cameras (test this first), `yolo` runs `ml_camera_harness.py` models on UDP 5600 frames (single camera only, so use `--cam-layout single`), `none` is lidar only.
- `--cam-layout dual` (default) is two cameras at yaw ∓FOV/2 (left is negative, clockwise-positive) with edges meeting at 0°. Set `--cam-fov` (70.4) and `--cam-width` (640) to match the bridge.
- Each box becomes a bearing wedge (same-class boxes cut at the seam are merged). Match = angular IoU × sqrt(cluster-extent prior × box-width prior); greedy one-to-one, thresholds `minOverlap`, `minMatch` via `--set`.
- Track class is sticky: votes change only when a box matches, so the class survives leaving camera view. Classes: person, bike, vehicle, pole/barrier; cones map to pole/barrier (YOLO has no cone class).
- Size priors are first guesses (`kPriors` in `lidarpipeline.cpp`).
- Extra metrics: class_coverage, class_accuracy, fused_rate. `--min-class-acc` / `--min-class-cov` set thresholds for `--score` with `--boxes gt`.
- Box error injection for stress tests: `--box-miss`, `--box-confuse`, `--box-jitter`.
- The CLI reads framed stdin (1 type byte + u32 LE length + payload): type 0 = raw STL-27L bytes, type 1 = JSON boxes `{"scan":N,"boxes":[{cam,u0,u1,cls,conf}]}`.
- Not built: homography range fallback for boxes with no cluster, §4.6 smoothing, ego-motion compensation.

Caveats:
- `Backend/lidarpipeline.*` is new and not yet wired into the dash or `CMakeLists.txt`. Thresholds in it are first guesses (see the header).
- Track position is the centroid of the visible points, so for large objects it sits on the near face, not the centre. Position error against CARLA's actor centre includes that bias.
- Velocities are relative to the kart (no ego-motion compensation) while the bridge's ground-truth velocities are world-frame, so compare them only with the kart stationary.
- CARLA ground truth has no occlusion information, so objects hidden behind others count as misses in live mode. The synthetic scene counts only objects with at least 3 ray hits.
- CARLA delivers partial rotations per tick; the harness merges the last 0.25 s of returns into one revolution, which a real STL-27L does not need.

## Lane assist (experimental)

The TwinLiteNet lane node and the closed-loop CARLA driver live in [experimental_lane_assist/](./experimental_lane_assist/README.md).

## Emergency braking and following (experimental)

Lidar-first braking/following with YOLO classes and the lane node's geometry: [experimental_emergency_braking_following/](./experimental_emergency_braking_following/README.md).
