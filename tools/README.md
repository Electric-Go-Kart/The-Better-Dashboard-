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
| `--mock` | off | Synthetic drive cycle, no CARLA needed |
