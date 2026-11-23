#!/usr/bin/env bash
# Closed-loop evaluation of the world-model planner in Isaac Lab: same seeds for every controller.
#   SIM_ROOT=/path/to/simulation bash closed_loop/run_ahmed_eval.sh [seeds.json] [out_dir] [parallel=1]
#
# SIM_ROOT is the simulation project (data collector, not in this repository yet): it must contain
# vendor/IsaacLab (Isaac Lab 3.0) and excavator_proto/collect_excavator_episode.py with the controller hooks
# described in closed_loop/README.md. BUNDLE overrides the planner bundle (default: $SIM_ROOT/models/...).
#
# modes (MODES="..."):
#   ahmed            planner.py (v1, WM + MPPI)
#   shield, mppi, reactive_vision   planner_v2.py modes
#   cvmpc            baselines.CVMPC on ground-truth machine parts
#   straight         naive go-to-goal, no avoidance
#   reactive         stop while the ground-truth machine clearance is < 2 m, else go to goal
# Command noise is off for every controller. HELDOUT=1 uses configs/test_heldout_v1.yaml (unseen skies / ground,
# more props, bigger excavators); PROPS=0 removes props. Every run also records chase.mp4 for review_video.py.
# Resumable: runs that already have metadata.json are skipped.
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=${SIM_ROOT:?set SIM_ROOT to the simulation project (see closed_loop/README.md)}
SEEDS=$(realpath ${1:-$HERE/seeds_ahmed20.json})
OUT=$(realpath -m ${2:-$PWD/closed_loop_runs}); PAR=${3:-1}
BUNDLE=${BUNDLE:-$ROOT/models/planner_bundle_vjepa2_1_vitl_g2.pt}
ENVARGS=""; [ "${HELDOUT:-0}" = "1" ] && ENVARGS="--config $HERE/configs/test_heldout_v1.yaml"
PROPARGS=""; [ "${PROPS:-1}" = "0" ] && PROPARGS="scene.props.enabled=false"
mkdir -p "$OUT"
python3 -c "import json;[print(r['seed'],r['scenario']) for r in json.load(open('$SEEDS'))]" > "$OUT/jobs.txt"
run() {
  seed=$1; scen=$2; mode=$3
  dst="$OUT/$mode/ep_$seed"; [ -f "$dst/metadata.json" ] && return
  case $mode in
    ahmed) extra="--ahmed-planner $BUNDLE" ;;
    reactive) extra="--reactive-stop-m 2.0" ;;
    shield|mppi|reactive_vision) extra="--ahmed-planner $BUNDLE --ahmed-version v2 --ahmed-mode $mode" ;;
    cvmpc) extra="--cvmpc" ;;
    *) extra="" ;;
  esac
  mkdir -p "$OUT/$mode"
  # Isaac can hang at start-up when instances launch together or after a killed run: launches are serialised with a
  # 30 s gap, stale Carbonite shared memory is cleaned first, and a run whose output folder has not appeared after
  # 10 min is killed (whole process group) and retried once.
  for attempt in 1 2; do
    ( flock 9; sleep 30; bash "$HERE/clean_stale_carb_shm.sh" ) 9>"$OUT/.launch.lock"
    (cd "$ROOT/vendor/IsaacLab" && exec setsid timeout 7200 ./isaaclab.sh -p ../../excavator_proto/collect_excavator_episode.py \
       --output "$dst" --seed $seed --behaviour naive --scenario $scen --split test \
       --set collection.branching.enabled=false "controllers.noise.scale=[0.0, 0.0]" controllers.noise.inject_rate_hz=0.0 $PROPARGS $ENVARGS \
       --chase-video $extra > "$OUT/$mode/ep_$seed.log" 2>&1) &
    pid=$!; waited=0
    while kill -0 $pid 2>/dev/null; do
      sleep 15; waited=$((waited + 15))
      if [ $waited -ge 600 ] && [ ! -d "$dst" ]; then
        kill -KILL -- -$pid 2>/dev/null; echo "$mode $seed start-up hang, killed (attempt $attempt)"; break
      fi
    done
    wait $pid 2>/dev/null
    [ -f "$dst/metadata.json" ] && break
  done
  echo "$mode $seed done: $(python3 -c "import json;print(json.load(open('$dst/metadata.json'))['termination'])" 2>/dev/null || echo FAILED)"
}
export -f run; export OUT ROOT HERE BUNDLE ENVARGS PROPARGS
for mode in ${MODES:-straight reactive shield}; do
  awk -v m=$mode '{print $1, $2, m}' "$OUT/jobs.txt" | xargs -P $PAR -L 1 bash -c 'run "$0" "$1" "$2"'
done
echo AHMED_EVAL_DONE
