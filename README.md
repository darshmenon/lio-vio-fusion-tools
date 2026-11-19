# fusion_tools
- `odom_to_tum.py bag topic out.tum` – bag -> TUM trajectory (source ROS2 first)
- `eval_traj.sh ref.tum est.tum...` – APE/RPE via evo (`~/.venvs/evo`)
- `imu_time_offset.py bag imu odom` – IMU vs odom time offset estimate
- `launch/fusion.launch.py` + `config/ekf_lio_wheel.yaml` – robot_localization EKF fusing LIO/VIO + wheel + IMU
Typical flow: extract LIO, VIO, fused and ground-truth to TUM, then run eval_traj.sh.
- `run_rosnav_lio_bench.sh [dur]` – record one Gazebo run with KISS-ICP, RKO-LIO and LIO-SAM live plus ground truth into a bag (own ROS domain, waits for a free machine)
