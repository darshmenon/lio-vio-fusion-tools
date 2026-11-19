# fusion_tools
Helper tools for LIO/VIO sensor fusion on ROS 2 Humble: extract trajectories from bags, score them, calibrate noise and timing,
fuse sources with honest covariances, and stress-test offline. Everything works on recorded bags, no simulator needed.

## Score
- `odom_to_tum.py bag topic out.tum` : bag -> TUM trajectory (Odometry or PoseStamped)
- `eval_traj.sh ref.tum est.tum...` : evo APE/RPE (`T_MAX` sets the time-match tolerance)
- `report.py --bag B --gt /gt --est name=/topic ... --out DIR [--after 0.5]` : one-step table + trajectory and error plots; `--after` scores a hold-out part; `name=topic@other_bag` or `name=file.tum` for estimates elsewhere

## Calibrate
- `calibrate_covariance.py` : wheel and LIO noise from a bag vs ground truth -> params file for the gate (`--before 0.5` calibrates on the first half only)
- `imu_time_offset.py bag imu odom` : IMU vs odometry time offset (`MODE=yaw` for legged robots)
- `calibrate_extrinsic.py bag imu odom` : IMU rotation extrinsic + time offset from gyro vs odometry angular velocity; says which axes are observable. Lever arm and camera-IMU: use Kalibr / LiDAR_IMU_Init

## Fuse
- `odom_health_gate.py` : sets covariances and down-weights a source that disagrees (adaptive variance); a LIO jump marks it untrusted, a stall does not
- `covariance_intersection.py` : CI fusion of two odometry sources with unknown correlation (ROS node, offline bag mode, selftest)
- `config/ekf_lio_wheel.yaml`, `config/ekf_lio_wheel_vx.yaml`, `launch/fusion.launch.py` : robot_localization EKF (LIO/VIO + wheel + IMU)
- `ekf_replay.sh bag out [lio_topic] [lio_frame]` : offline EKF fusion (`GATE=1`, `GATE_PARAMS=`)

## Stress-test (no Gazebo)
- `replay_lio.sh bag out rko|kiss|both [-- k:=v]` : replay a bag through RKO-LIO / KISS-ICP
- `inject_faults.py in out --drop-imu/--imu-shift/--imu-noise/--blank-lidar` : corrupt a bag
- `run_sweeps.sh bag out` : parameter sweep + fault replays
- `run_rosnav_lio_bench.sh [dur]` : record one Gazebo run with KISS-ICP, RKO-LIO and LIO-SAM live (own ROS domain, waits for a free machine)

## Findings so far (rosnav cafe weave, 158 s; Go2 arena and outdoor)
- Scoring against ground truth: KISS-ICP APE RMSE 0.027 m, RKO-LIO 0.045 m, wheel odometry 5.3 m, LIO-SAM diverged on fast turns
- The sim wheel odometry published zero covariance, so an EKF trusted it fully; calibrated noise is 0.16 m/s forward speed and 0.32 rad/s yaw rate
- Fusion, held-out second half: KISS-ICP alone 0.030 m; EKF with raw wheel odometry 0.075 m; EKF with calibrated adaptive gate 0.053 m (end drift 2.4% -> 0.6%)
- Legged robots: gyro-norm time offset is unreliable (correlation 0.2); use `MODE=yaw` (about +0.055 s on the Go2)
