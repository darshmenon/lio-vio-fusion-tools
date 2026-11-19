#!/usr/bin/env bash
# Offline EKF fusion of a LIO odom + wheel odom + IMU from a recorded bag (no Gazebo). Records /fusion/odom.
# Usage: ekf_replay.sh IN_BAG OUT_DIR [LIO_TOPIC=/kiss/odometry] [LIO_FRAME=odom_lidar]
# Env: GATE_PARAMS=<yaml from calibrate_covariance.py> GATE=1 (insert odom_health_gate.py: honest covariances + down-weight bad sources) CONFIG=<yaml> RATE=1.0 DOMAIN=93 WHEEL=/odom IMU=/imu IMU_FRAME=imu_link
set -u
IN=$1; OUT=$2; LIO=${3:-/kiss/odometry}; LFRAME=${4:-odom_lidar}
export ROS_DOMAIN_ID=${DOMAIN:-93}; HERE=$(cd "$(dirname "$0")" && pwd)
set +u; source /opt/ros/humble/setup.bash; set -u
dom_pids(){ for d in /proc/[0-9]*; do q=${d#/proc/}; [ "$q" = "$$" ] && continue
  tr '\0' '\n' < $d/environ 2>/dev/null | grep -qx "ROS_DOMAIN_ID=$ROS_DOMAIN_ID" && echo $q; done; } 2>/dev/null
[ -n "$(dom_pids)" ] && { echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID already in use by another process; pick DOMAIN=<other>"; exit 1; }
mkdir -p "$OUT"; PIDS=()
# stop recorder cleanly first, then kill every process of THIS domain (children of setsid'd launches included)
trap 'for p in "${PIDS[@]}"; do kill -INT -- -$p 2>/dev/null; done; sleep 3; for q in $(dom_pids); do kill -9 $q 2>/dev/null; done' EXIT
SIM="--ros-args -p use_sim_time:=true"
# identity bridges: LIO's own odom frame -> odom, IMU frame -> base_link (the recording has no TF)
setsid ros2 run tf2_ros static_transform_publisher --frame-id odom --child-frame-id $LFRAME $SIM > /dev/null 2>&1 < /dev/null & PIDS+=($!)
setsid ros2 run tf2_ros static_transform_publisher --frame-id base_link --child-frame-id ${IMU_FRAME:-imu_link} $SIM > /dev/null 2>&1 < /dev/null & PIDS+=($!)
EKF_LIO=$LIO; EKF_WHEEL=${WHEEL:-/odom}
if [ "${GATE:-0}" = 1 ]; then
  setsid python3 "$HERE/odom_health_gate.py" --ros-args -p use_sim_time:=true -p lio_in:=$LIO -p wheel_in:=${WHEEL:-/odom} ${GATE_PARAMS:+--params-file $GATE_PARAMS} > "$OUT/gate.log" 2>&1 < /dev/null & PIDS+=($!)
  EKF_LIO=/gated/lio; EKF_WHEEL=/gated/wheel
fi
setsid ros2 run robot_localization ekf_node --ros-args --params-file "${CONFIG:-$HERE/config/ekf_lio_wheel.yaml}" \
  -p use_sim_time:=true -p publish_tf:=false -p odom0:=$EKF_LIO -p odom1:=$EKF_WHEEL -p imu0:=${IMU:-/imu} \
  -r odometry/filtered:=/fusion/odom -r __node:=ekf_filter_node > "$OUT/ekf.log" 2>&1 < /dev/null & PIDS+=($!)
sleep 3
setsid ros2 bag record -o "$OUT/fusion_bag" --use-sim-time /fusion/odom > "$OUT/rec.log" 2>&1 < /dev/null & PIDS+=($!)
sleep 3
ros2 bag play "$IN" --clock --rate ${RATE:-1.0} --topics $LIO ${WHEEL:-/odom} ${IMU:-/imu} > "$OUT/play.log" 2>&1
sleep 3; echo "fusion odom in $OUT/fusion_bag"
