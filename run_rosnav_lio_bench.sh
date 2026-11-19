#!/usr/bin/env bash
# Record one rosnav cafe run with KISS-ICP + RKO-LIO + LIO-SAM live, dynamic motion, into a bag.
# Writes only under $OUT. Own ROS_DOMAIN_ID/GZ_PARTITION. Waits for no foreign gz sim and load<6.
# Usage: run_rosnav_lio_bench.sh [DUR=150]
set -u
DUR=${1:-150}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-91}; export GZ_PARTITION=fusion_$ROS_DOMAIN_ID
OUT=${OUT:-$HOME/fusion_tools/shared/from_fusion/rosnav_$(date +%Y%m%d_%H%M)}; mkdir -p "$OUT"
log(){ echo "[fusion_bench $(date +%T)] $*" | tee -a "$OUT/run.log"; }
log "waiting for free machine (no gz sim, load<6)"
while pgrep -f "gz sim" >/dev/null || [ "$(awk '{print int($1)}' /proc/loadavg)" -ge 6 ]; do sleep 20; done
set +u; source /opt/ros/humble/setup.bash; source "$HOME/rosnav/install/setup.bash"; set -u
PIDS=()
mine_kill(){  # kill only processes carrying THIS run's GZ_PARTITION
  for d in /proc/[0-9]*; do p=${d#/proc/}
    tr '\0' '\n' < $d/environ 2>/dev/null | grep -qx "GZ_PARTITION=$GZ_PARTITION" && kill -9 "$p" 2>/dev/null
  done; }
trap 'for p in "${PIDS[@]}"; do kill -9 -- -$p 2>/dev/null; done; mine_kill' EXIT
log "domain $ROS_DOMAIN_ID partition $GZ_PARTITION, out $OUT"
setsid timeout $((DUR+150)) ros2 launch rosnav_bot slam_nav.launch.py world_name:=cafe lidar_type:=3d slam:=false headless:=true rviz:=False > "$OUT/sim.log" 2>&1 < /dev/null & PIDS+=($!)
sleep 60
setsid ros2 launch kiss_icp odometry.launch.py topic:=/points base_frame:=base_link publish_odom_tf:=false visualize:=false use_sim_time:=true > "$OUT/kiss.log" 2>&1 < /dev/null & PIDS+=($!)
setsid ros2 launch rko_lio odometry.launch.py imu_topic:=/imu lidar_topic:=/points base_frame:=base_link odom_frame:=rko_odom autodetect:=false deskew:=false use_sim_time:=true > "$OUT/rko.log" 2>&1 < /dev/null & PIDS+=($!)
( set +u; source "$HOME/lio_ws/install/setup.bash"; set -u
  setsid ros2 launch rosnav_bot lio_sam_sim.launch.py > "$OUT/liosam.log" 2>&1 < /dev/null & echo $! > "$OUT/liosam.pid" ); PIDS+=($(cat "$OUT/liosam.pid"))
setsid ros2 run ros_gz_bridge parameter_bridge "/model/diff_drive/odometry_with_covariance@nav_msgs/msg/Odometry[gz.msgs.OdometryWithCovariance" --ros-args -r /model/diff_drive/odometry_with_covariance:=/gt_odom > "$OUT/gt.log" 2>&1 < /dev/null & PIDS+=($!)
sleep 15
setsid ros2 bag record -o "$OUT/bag" --use-sim-time /clock /imu /points /gt_odom /odom /kiss/odometry /rko_lio/odom /lio_sam/mapping/odometry > "$OUT/bag.log" 2>&1 < /dev/null & PIDS+=($!)
sleep 5
log "driving weaving motion for ${DUR}s (faster, sharp turns to excite the IMU)"
end=$((SECONDS+DUR)); sgn=1
while [ $SECONDS -lt $end ]; do
  timeout 4 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: $(echo "$sgn*0.8" | bc -l)}}" >/dev/null 2>&1
  sgn=$((-sgn))
done
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1
sleep 3; kill -INT "${PIDS[-1]}" 2>/dev/null; sleep 5
log "done. bag: $OUT/bag"; ros2 bag info "$OUT/bag" 2>&1 | tee -a "$OUT/run.log" | head -20
