#!/usr/bin/env bash
# Compare estimated trajectories against a reference with evo (APE + RPE).
# Usage: eval_traj.sh <reference.tum> <est1.tum> [est2.tum ...]
set -e
EVO=${EVO_BIN:-$HOME/.venvs/evo/bin}
T=${T_MAX:-0.02}   # max timestamp difference for matching poses (s)
ref=$1; shift
[ -f "$ref" ] && [ $# -ge 1 ] || { sed -n '2,3p' "$0"; exit 1; }
for est in "$@"; do
  echo "=== $est: APE (translation, SE3-aligned) ==="
  $EVO/evo_ape tum "$ref" "$est" -a --t_max_diff $T | grep -E "rmse|mean|max"
  echo "=== $est: RPE (1 m segments) ==="
  $EVO/evo_rpe tum "$ref" "$est" -a -d 1 -u m --t_max_diff $T | grep -E "rmse|mean|max"
done
echo "Plot: $EVO/evo_traj tum $* --ref $ref -a -p"
