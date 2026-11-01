#!/bin/zsh
# run_equiv.sh <tree> <tip tree> <out dir> <route dir prefix> [group ...]: equiv_replay.py on each segment group (default: the
# route 00002310 engaged-stop groups), at most 4 at a time. Activate the openpilot venv first. EQUIV_TOGGLES_JSON and EQUIV_WITH_GAS
# pass through. Example: run_equiv.sh $PWD ../subtract-wt /tmp/out ~/.route_sync/data/media/0/realdata/00002310--81789e4237 "14 15"
set -u
TOOLS=${0:A:h}
TREE=${1:A} TIP=${2:A} OUT=${3:A} ROUTE=$4
shift 4
groups=("$@")
(( $#groups )) || groups=("14 15" "16 17 18" "20 21" "24 25" "28 29 30")
mkdir -p $OUT
: > $OUT/rc.txt
cd $TREE
export PYTHONPATH=$TREE:$TREE/frogpilot/third_party
for g in $groups; do
  name=g${g// /_}
  segs=()
  for s in ${=g}; do segs+=($ROUTE--$s); done
  ( python $TOOLS/equiv_replay.py $OUT/$name.pkl $TIP $segs > $OUT/$name.log 2>&1; echo "$name rc=$?" >> $OUT/rc.txt ) &
  while (( $(jobs -r | wc -l) >= 4 )); do sleep 1; done
done
wait
sort $OUT/rc.txt
