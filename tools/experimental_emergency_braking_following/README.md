# Experimental emergency braking and following

Lidar is the primary sensor. It finds, tracks and predicts everything in front of the kart and decides when to
slow, follow or emergency-brake. YOLO-FastestV2 only says *what* a lidar track is (person / bike / vehicle), which
changes how far the planner assumes it may move. The lane node (TwinLiteNet) supplies the lane geometry: the kart
keeps steering with pure pursuit, and the planner uses the lane to decide which objects are in the kart's path and
to nudge a few cm away from things beside it.

| File | Role | Venv |
|---|---|---|
| `eb_core.py` | clustering, tracker (constant-velocity Kalman), class voting, planner, speed controller (numpy only) | any |
| `eb_sim.py` | offline 2D simulator + scenario self-test | any |
| `eb_vision_node.py` | YOLO on camera UDP 5606 -> boxes JSON on UDP 5607 | `.venv-ml` |
| `eb_drive.py` | CARLA driver: lidar, scenarios, steering, brake/throttle, log + summary | `.venv-carla` |

## How it decides

1. Lidar returns from the last 0.15 s are ego-motion compensated (wheel speed + yaw rate), clustered and tracked.
2. Vision boxes are matched to tracks by bearing overlap and vote a class (boxes are ~0.25 s old, which is fine
   because the class only needs to be right once).
3. A track inside the kart's lane corridor is a **leader**: follow with a speed-dependent gap (`d_min` 1.5 m + 1.2 s).
4. Any other track is predicted forward 5 s. Contact with the corridor -> speed cap (comfortable decel) and, if the
   decel needed reaches 3 m/s², **AEB** (held 0.5 s). People are assumed able to swerve more than cars, so their
   "might step in" zone is larger; this slows the kart to a creep rather than a stop.
5. No lidar for 0.4 s -> stop. No lane estimate for 0.5 s -> stop (lidar braking still works without vision).

## Offline check (no CARLA, no GPU)

    python3 eb_sim.py --selftest
    python3 eb_sim.py --scenario ped-cross      # per-second trace

Verified: static car at 5 m/s stops with 2 m to spare, slow lead followed at its speed, crossing pedestrian triggers
AEB, a pedestrian walking along the road edge does not, oncoming traffic is ignored, a lidar dropout stops the kart,
and every one of these also passes with vision off. A sweep of 24 pedestrian cases (3-7 m/s) had 3 contacts, all
where the pedestrian steps out less than ~3-4 m ahead, inside the braking distance. A car cutting across the lane
about 3 m ahead also hits. Those are limits of the kart's braking, not of the logic.

## CARLA

Three terminals, in this order:

    # 1 (.venv-ml)  lane geometry
    python ../experimental_lane_assist/seg_lane_node.py --no-display --device cuda
    # 2 (.venv-ml)  classes
    python eb_vision_node.py --device cuda
    # 3 (.venv-carla)
    python eb_drive.py --scenario ped-cross --spawn 6 --speed 4 --out runs/ped1

Scenarios: `none`, `lead-stop` (parked car ahead), `lead-slow` (car at `--lead-speed`), `ped-cross` (steps out when
the kart is within `--ped-trigger` m), `ped-sidewalk` (walks along the road edge: must NOT cause braking).
`--spawn-cars N --spawn-walkers N` add background traffic.

Useful flags: `--truth-lane` (steer from the map, so you can test braking without the lane node), `--no-vision`
(lidar only), `--lidar-noise`, `--lidar-drop` (harder scans), `--no-nudge`, `--lidar-x` (mount position; bumper
distances come from the vehicle's bounding box).

`runs/.../drive.csv` logs state, target speed, required decel, gap, tracked classes and the ground-truth distance
to the scenario object; `summary.txt` reports collisions, closest approach, AEB time and lane RMS.

Ports: camera 5600 (lane node), 5606 (vision node), lane state 5604, boxes 5607, odometry 5603, debug view 5608.

## Seeing what YOLO sees and what is matched

- `python eb_vision_node.py --device cuda --show` opens a window with the camera frame and the boxes being sent
  (the console line every 5 s shows ms/frame and fps).
- `python eb_view.py` (`.venv-ml`, start it any time) shows a top-down view fed by `eb_drive.py`: lidar points, each
  track coloured by its matched class with its id, class votes and velocity arrow, the YOLO boxes as coloured
  bearing lines at the top, the bearing span of each track (short lines at the kart), the lane path and corridor,
  the track the planner is reacting to (red circle), and state / target speed / required decel / gap. A track is
  matched to a box when the box's bearing range overlaps the track's span; grey = no class yet (treated as
  person-like). `--save-dir DIR` writes a PNG per frame.
- `drive.csv` also logs the classes of the tracks each tick.
`seg_lane_node.py` now also sends `width` in its state message (used for the nudge room).

## Testing on a Pi with CARLA as the sensors

CARLA and `eb_drive.py` stay on the desktop; the Pi runs the two ML nodes on the camera stream. UDP carries the
camera and wheel speeds to the Pi and the lane state and boxes back, so the latency you see includes the network.
The planner still runs on the desktop (it is cheap numpy), so this tests what is expensive on a Pi: YOLO and
TwinLiteNet speed and the effect of their delay on driving.

    # Pi (use the ncnn backend; copy the .param/.bin files)
    python ../experimental_lane_assist/seg_lane_node.py --no-display --backend ncnn --seg-param s.param --seg-bin s.bin \
        --state-udp DESKTOP_IP:5604
    python eb_vision_node.py --backend ncnn --yolo-param y.param --yolo-bin y.bin --out-host DESKTOP_IP
    # desktop
    python eb_drive.py --pi-host PI_IP --scenario ped-cross --spawn 6 --out runs/pi1

Use a wired link (Wi-Fi jitter would be blamed on the Pi), and check the Pi's `ms/frame` lines against the
`lane_age` / camera-box timing in `drive.csv`. Compare the same scenario with `--pi-host` unset (desktop GPU) to
see what the Pi's slower inference costs. The lidar never leaves the desktop: the planner reads it in-process, so
lidar braking is unaffected by the Pi, and `--no-vision` or killing the vision node shows the lidar-only fallback.
This older `eb_drive.py --pi-host` path does **not** run the planner on the Pi. For the full experimental split,
use the separate scripts below.

## Full CARLA / Pi split (experimental)

The desktop runs CARLA, the dashboard and `carla_split.py`. The Pi runs the lane and YOLO nodes and
`pi_planner.py`: lidar clustering/tracking, class fusion, speed planning and steering all happen on the Pi.
The desktop forwards the Pi's fused objects to the dashboard's localhost detection feed; it displays the
Pi's recommendations in the existing `eb_view.py` viewer. The normal dashboard telemetry bridge can still
provide speed, camera and lidar, but must not publish its ground-truth detections at the same time.

**Simulation only.** No Pi process in this split opens CAN or controls an actuator. The CARLA bridge invokes
`VehicleControl` only on the simulated hero. This is experimental demo code, not a safety system. CARLA
autopilot is not a safety guarantee, and has not been validated as a safe fallback here.

### Prepare

1. Connect the Pi and desktop over a trusted wired LAN; find their actual addresses. UDP has no authentication
   or encryption: do not expose these ports to an untrusted network.
2. Keep the existing Pi ML environment (`~/ml-env`) with numpy, OpenCV and ncnn, and copy the updated `tools`
   tree to the Pi. For example, from the project root (without replacing Pi model files):

       rsync -av --exclude='.venv*' --exclude='__pycache__/' --exclude='twinlite_ncnn/out/' \
           tools/ user@pi-4-test:~/gokart-tools/

   Ensure the Pi has the 640x384 Nano ncnn `.param` and `.bin`, the YOLO ncnn `.param` and `.bin`, and
   YOLO `data/coco.data`. The example below uses the previously copied `nano_fp16` files; adjust paths to
   your actual files. The YOLO output names for the supplied optimized model are `794,796`.
3. Start CARLA and create **exactly one** vehicle with role name `hero`. The split attaches to it without
   repositioning or destroying it. For shadow mode, use a manual client that can attach to that existing hero.
   The standard CARLA `manual_control.py` may spawn its own hero and may not support attaching without changes.
   For shadow mode, you can instead start it first and let the split attach to the hero it spawned. Do not run
   two clients that both write controls to the same hero.
4. Start the dashboard and its existing **virtual CAN** setup as described in [tools/README.md](../README.md).
   On the desktop, run the usual telemetry bridge with detections disabled, so ground-truth objects do not
   overwrite Pi detections:

       cd /home/joshua/Desktop/gokart/dashboard/The-Better-Dashboard-
       tools/.venv-carla/bin/python tools/carla_vesc_bridge.py --iface vcan0 --no-detections \
           --lidar-x 1.0 --lidar-z 0.5 --cam-width 640 --cam-height 360 --cam-fov 70.4

   This telemetry bridge attaches its own camera/lidar for the dashboard; the split attaches separate sensors
   for the Pi. Do not point this demo at a physical CAN interface.

### Start the Pi processes

Use three terminals, activate `~/ml-env` in each, and replace `DESKTOP_IP` and `YOLO_REPO` with their actual
values. `YOLO_REPO` is the Pi directory containing `data/coco.data` and
`sample/ncnn/model/yolo-fastestv2-opt.param` / `.bin`.

    cd ~/gokart-tools
    python experimental_lane_assist/seg_lane_node.py --backend ncnn --no-display --latest-frame \
        --seg-param twinlite_ncnn/out/nano_fp16.param --seg-bin twinlite_ncnn/out/nano_fp16.bin \
        --seg-width 640 --seg-height 384 --seg-letterbox \
        --cam-width 640 --cam-height 360 --cam-fov 70.4 --cam-x 0.8 --cam-height-m 1.5 \
        --state-udp 127.0.0.1:5604

    cd ~/gokart-tools
    python experimental_emergency_braking_following/eb_vision_node.py --backend ncnn --latest-frame \
        --yolo-repo YOLO_REPO \
        --yolo-param YOLO_REPO/sample/ncnn/model/yolo-fastestv2-opt.param \
        --yolo-bin YOLO_REPO/sample/ncnn/model/yolo-fastestv2-opt.bin --yolo-out 794,796

    cd ~/gokart-tools
    python experimental_emergency_braking_following/pi_planner.py --desktop-host DESKTOP_IP

`--latest-frame` drains queued camera datagrams before inference; it reduces backlog but does not prove that
the selected frame is current. The lane node's camera geometry must match the CARLA split's `--cam-*` values.
The model size must match its fixed ncnn export; the example opts into letterboxing and removes padding from
the resulting masks. Without `--seg-letterbox`, the previous stretched preprocessing remains the default.

### Start the desktop bridge and viewer

In another desktop terminal, after the hero and Pi processes are running:

    cd /home/joshua/Desktop/gokart/dashboard/The-Better-Dashboard-
    tools/.venv-carla/bin/python tools/experimental_emergency_braking_following/carla_split.py \
        --pi-host PI_IP --mode shadow --duration 600 --scenario none

For the debug window:

    cd /home/joshua/Desktop/gokart/dashboard/The-Better-Dashboard-
    .venv-ml/bin/python tools/experimental_emergency_braking_following/eb_view.py

Shadow mode never changes the hero's steering, pedals or autopilot, including on exit. The viewer shows the
Pi's recommended steering, throttle and brake, tracks, lane, planner state and locally measured debug-link age.
It marks the link stale after one second without packets (`--stale-after` changes that).

To let the Pi drive the **simulated** hero, stop any manual client from writing controls and change to
`--mode simulated`. With no fresh valid command the desktop enables CARLA Traffic Manager autopilot; when a
valid command returns it disables autopilot and resumes applying Pi commands automatically. On normal exit it
leaves simulated autopilot enabled. Autopilot can use its own speeds and route, not the Pi's `--speed`. Do not
run this mode if another CARLA client is also writing vehicle controls. Ensure the hero remains alive when the
manual client stops: some clients destroy vehicles they spawned on exit. `--scenario lead-stop`, `lead-slow`,
`ped-cross` or `ped-sidewalk` add the existing scenario actors relative to the hero; `--dist`, `--ped-trigger`
and `--lead-speed` tune their placement or motion. `--speed 2.24` is the default Pi target in m/s, approximately
5 mph; actual speed and autopilot speed are not guaranteed. The script destroys only its own sensors and
scenario actors on exit.

### Ports, timing and checks

| Destination | UDP port | Data |
|---|---:|---|
| Pi | 5600, 5606 | JPEG camera for lane, YOLO respectively |
| Pi | 5603, 5604, 5607 | wheel odometry, local lane state, local boxes |
| Pi | 5611, 5612 | lidar scan, ego state and geometry |
| Desktop | 5613, 5608 | commands/fused tracks, viewer debug data |
| Desktop loopback | 5602 | forwarded fused dashboard objects |

Allow the needed incoming UDP ports in each host's firewall. Lane/boxes/odometry remain local to the Pi; the
viewer listens on the desktop. `--no-dashboard-detections` disables forwarding. Do not run another producer
on the same dashboard detection port. If the split is stopped normally or the Pi command expires, forwarded
objects are cleared; abrupt termination of the desktop bridge cannot send a clear.

The desktop uses its monotonic clock and an echoed ego sequence to reject old, reordered, wrong-session and
out-of-range commands. Its default `--timeout 0.5` expires based on when that ego sample was **sent**, not on
the clocks on either host. Restarting the Pi planner uses a new instance ID and can resume; old-instance
packets are rejected. The Pi separately times out locally received ego, lane, boxes and lidar. A missing lane
drives the Pi target speed to zero, and missing lidar produces `LIDAR-LOST`; these are planner behaviors,
not independent safety guarantees. Camera, lane, boxes and lidar datagrams have no end-to-end capture-age
verification or synchronized frame IDs, and UDP packets may be dropped, delayed or reordered. In particular,
a recent Pi command is not proof that its underlying camera frame was recent.

Start with shadow mode, confirm viewer updates and fused detections, and stop the Pi planner to check that the
viewer marks the link stale. Only then try simulated mode in a clear CARLA scene: stop/restart the planner and
observe the desktop's autopilot/resume messages. The split uses separate processes and UDP on each host; do not
run the two-machine example as-is with both hosts on localhost because camera ports can conflict.

## Limitations

- The CARLA scripts were written without access to a CARLA server here: only the core, the simulator and a compile
  check were run. Expect to tune `--lidar-x`, the speed controller gains and the scenario placement on first run.
- Lidar is a single plane. Anything below or above it (curbs, low obstacles) is invisible; the CARLA lidar sees the
  hero's own body unless it is mounted clear of it (the planner masks the kart footprint).
- Velocity comes from track centroids, so it is noisy for large or half-visible objects. Moving objects are smeared
  by the 0.15 s window.
- The corridor assumes the kart returns to the lane centre within ~6 m.
- YOLO-FastestV2 misses small or distant people; the lidar still brakes for them, only with the generic "unknown"
  (person-like) margin.
