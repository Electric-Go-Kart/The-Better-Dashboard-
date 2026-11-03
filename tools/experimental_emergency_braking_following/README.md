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
To also run the planner on the Pi you would need to split `eb_drive.py`'s loop out; that is not done.

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
