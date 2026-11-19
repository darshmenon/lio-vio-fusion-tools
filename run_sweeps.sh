#!/usr/bin/env bash
# Offline RKO-LIO parameter sweep + fault-injection replays on a recorded bag (no Gazebo).
# Usage: run_sweeps.sh IN_BAG OUT_DIR      (ROS_DOMAIN_ID 92; one replay at a time)
set -u
IN=$1; OUT=$2; HERE=$(cd "$(dirname "$0")" && pwd); mkdir -p "$OUT"
set +u; source /opt/ros/humble/setup.bash; set -u
run(){ name=$1; bag=$2; shift 2; echo "[$(date +%T)] replay $name"; "$HERE/replay_lio.sh" "$bag" "$OUT/$name" rko -- "$@" > "$OUT/$name.log" 2>&1; }
run base "$IN"
run voxel05 "$IN" voxel_size:=0.5
run voxel10 "$IN" voxel_size:=1.0
for f in "drop30:--drop-imu 0.3" "shift100ms:--imu-shift 0.1" "blank40_60:--blank-lidar 40,60"; do
  n=${f%%:*}; args=${f#*:}
  python3 "$HERE/inject_faults.py" "$IN" "$OUT/fault_$n" $args > "$OUT/inject_$n.log" 2>&1
  run "fault_$n" "$OUT/fault_$n"
  rm -rf "$OUT/fault_$n"   # keep disk small; est bag is kept
done
echo "[$(date +%T)] sweeps done"
