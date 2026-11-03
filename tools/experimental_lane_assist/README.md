# Experimental lane assist

Run everything from `tools/` or this folder. The scripts import `carla_vesc_bridge.py` and `ml_camera_harness.py` from `tools/`; use `../.venv-ml` for the node and `../.venv-carla` for the driver. Run outputs go in `runs/` (git-ignored).

## Lane offset from TwinLiteNet masks (seg_lane_node.py)

[seg_lane_node.py](./seg_lane_node.py) turns TwinLiteNet's lane-line and drivable masks into a lateral offset and heading error
in the kart frame (x forward, y right, m). Camera frames only, no CARLA ground truth. It uses a flat-ground homography
from the camera mount (`--cam-x`, `--cam-height-m`, `--cam-pitch`, `--cam-fov`, `--cam-width/height`), which must match the
bridge (`--cam-x`, `--cam-z`, `--cam-fov`, frame size).

```bash
python seg_lane_node.py --selftest        # homography + estimator check on synthetic masks, no model needed
python seg_lane_node.py                   # live: bridge streaming 640x360 on UDP 5600
python seg_lane_node.py --folder frames --loop --csv runs/lane.csv
python seg_lane_node.py --ros             # also publish /lane/state (std_msgs/Float32MultiArray)
python seg_lane_node.py --ros --ros-image /carla/ego/rgb_front/image   # frames from a ROS topic
```

- Right-hand traffic: it tracks the kart's own lane as a left boundary (nearest line left, normally the centre line) and a right boundary (nearest line right, else the right edge of the drivable mask), each fitted over the sample rows (`--rows`). The drivable mask is never used for its middle, because it includes the oncoming lane. A boundary that vanishes (dashed centre line gap) is remembered for `--line-hold` s, then the centre comes from the remaining boundary plus the learned lane width (`--lane-width` is the starting value). Short loss of everything is held for `--hold` s, then reported invalid.
- Stop lines and crosswalks: TwinLiteNet marks them as lane lines too. The line mask is warped to a top-down view and each blob is checked. Transverse or very wide blobs (stop lines, ladder bars, `--max-line-angle`, `--max-blob-width`) are dropped, as are groups of three or more short stripes side by side (zebra crosswalks, `--max-stripe-len`, `--stripe-gap`). Sample rows overlapping a dropped mark give no line, lines must be within `--gate` m of the remembered boundary, and lane-width learning is paused and limited to 0.7-1.3x `--lane-width`. The window shows `[marks ignored]`, kept line pixels in red and dropped ones in orange in the top-down panel.
- `/lane/state` data: `[valid, offset_m, heading_rad, lookahead_y_m, confidence]`. Positive offset = lane centre is to the right of the kart.
- Window: camera with masks, sample points (yellow = both lines, orange = one line, blue = road mask, grey = rejected), fit in magenta, and a top-down view. `q` quit, `s` save PNG.
- Only the selftest, geometry and renderer were checked. The torch/NCNN model path, UDP, and ROS (`rclpy`) paths are untested.
- The homography assumes flat ground and a fixed pitch. Slopes, kart roll and braking pitch will bias the far rows.

Wheel odometry (lane node): the bridge streams left/right wheel speeds (m/s, the same values as the VESC frames, plus `--odom-noise`) on UDP `--odom-port` (5603, 0 = off). `seg_lane_node.py` integrates them (`yaw rate = (v_left - v_right) / --track-width`, set it to the bridge `--track`) and moves the remembered lane boundaries with the kart, so dashed gaps and crosswalks no longer freeze the lane. While odometry is live the memory lasts `--line-hold-odom` s (4) instead of `--line-hold` (1.5). With no stream it falls back to the old behaviour. Wheel slip and scrub in turns bias the yaw rate, so long blind stretches in curves still drift. The same pose interface (`est.update(..., pose)`) is where an IMU gyro would plug in later.

Closed-loop test ([lane_drive.py](./lane_drive.py)): spawns (or finds) the `hero` vehicle, streams its camera and wheel speeds like the bridge, and steers using the selected controller and the lane node's offset, heading, and curvature estimate (UDP 5604 from `seg_lane_node.py`). CARLA's map is used only for the log: `drive.csv` has the estimate next to the true lane offset/heading, and `summary.txt` has kart lateral error, lane departures, estimator error, valid time and lost-lane stops. Run in two terminals, with CARLA already up and the bridge NOT running (the driver uses ports 5600 and 5603):
```bash
../.venv-carla/bin/python lane_drive.py --duration 120 --out runs/run1 [--map Town03 --spawn 5 --speed 3]
../.venv-ml/bin/python seg_lane_node.py --no-display --save-dir runs/run1/frames --csv runs/run1/lane.csv
```

`--truth-drive` steers from the map instead, to check the harness. Stops when the kart is more than 3 m from the lane centre. Traffic lights are forced green (`--keep-lights` to disable). There is no turning logic, so junctions are driven straight or reported lost.

### Controller comparison

`lane_drive.py` supports four steering controllers over the same lane estimate: `pure-pursuit` (default), `stanley`, `pid`, and a kinematic-bicycle `mpc`. Each run writes the controller name into `drive.csv` and `summary.txt`. For a fair comparison, keep CARLA map, spawn point, speed, lane-node settings and duration identical, and run each controller from a fresh start. Use separate output folders:

```bash
../.venv-carla/bin/python lane_drive.py --controller pure-pursuit --duration 120 --spawn 5 --speed 3 --out runs/compare-pure-pursuit
../.venv-carla/bin/python lane_drive.py --controller stanley      --duration 120 --spawn 5 --speed 3 --out runs/compare-stanley
../.venv-carla/bin/python lane_drive.py --controller pid          --duration 120 --spawn 5 --speed 3 --out runs/compare-pid
../.venv-carla/bin/python lane_drive.py --controller mpc          --duration 120 --spawn 5 --speed 3 --out runs/compare-mpc
../.venv-carla/bin/python compare_lane_drives.py runs/compare-pure-pursuit runs/compare-stanley runs/compare-pid runs/compare-mpc
```

Keep the lane-node process running against the active driver's camera stream, restarting it between runs if its lane state does not reconnect. Compare the `drive:` RMS lateral error, `lane departures`, lane-node valid percentage, and lost-lane stop percentage in each `summary.txt`; lower lateral error/departures and higher valid time are preferable. `drive.csv` preserves per-sample truth and estimate for plotting. The PID and Stanley gains and MPC horizon/search width can be adjusted with `--pid-*`, `--stanley-*`, and `--mpc-*`; first compare the defaults, then tune with the same route and conditions. The MPC uses a small bounded beam search and is computationally heavier than the other controllers. These are simulation experiments, not safety-validated lane-assist controllers.

`compare_lane_drives.py` prints the same map-truth lateral RMS, maximum error, departure rate (|offset| > 1.5 m), valid-estimate rate, and lost-lane stop rate in one table. Its positional arguments are run directories, so they can be reordered or extended to compare tuned runs as well.
