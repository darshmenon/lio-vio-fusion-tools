#!/usr/bin/env bash
# Replay a recorded bag through RKO-LIO and/or KISS-ICP offline (no Gazebo), record estimates into a new bag.
# Usage: replay_lio.sh IN_BAG OUT_DIR [rko|kiss|both] [-- extra rko params as k:=v ...]
# Env: LIDAR=/points IMU=/imu BASE=base_link DOMAIN=92 RATE=0.5 (play speed; lower = safer under load)
set -u
IN=$1; OUT=$2; WHICH=${3:-rko}; shift $(( $# < 3 ? $# : 3 )); [ "${1:-}" = "--" ] && shift
export ROS_DOMAIN_ID=${DOMAIN:-92}
LIDAR=${LIDAR:-/points}; IMU=${IMU:-/imu}; BASE=${BASE:-base_link}
set +u; source /opt/ros/humble/setup.bash; source "$HOME/rosnav/install/setup.bash"; set -u
mkdir -p "$OUT"; PIDS=()
trap 'for p in "${PIDS[@]}"; do kill -INT -- -$p 2>/dev/null; done; sleep 3' EXIT
REC=""
if [ "$WHICH" != kiss ]; then
  setsid ros2 launch rko_lio odometry.launch.py imu_topic:=$IMU lidar_topic:=$LIDAR base_frame:=$BASE odom_frame:=rko_odom \
     autodetect:=false deskew:=false use_sim_time:=true "$@" > "$OUT/rko.log" 2>&1 < /dev/null & PIDS+=($!); REC="$REC /rko_lio/odom"
fi
if [ "$WHICH" != rko ]; then
  setsid ros2 launch kiss_icp odometry.launch.py topic:=$LIDAR base_frame:=$BASE publish_odom_tf:=false visualize:=false \
     use_sim_time:=true > "$OUT/kiss.log" 2>&1 < /dev/null & PIDS+=($!); REC="$REC /kiss/odometry"
fi
sleep 5
setsid ros2 bag record -o "$OUT/est_bag" --use-sim-time $REC > "$OUT/rec.log" 2>&1 < /dev/null & PIDS+=($!)
sleep 3
ros2 bag play "$IN" --clock --rate ${RATE:-0.5} --topics $LIDAR $IMU /gt_odom /odom > "$OUT/play.log" 2>&1
sleep 3
echo "estimates recorded in $OUT/est_bag (also need gt from $IN)"
