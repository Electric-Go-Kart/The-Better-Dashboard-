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
