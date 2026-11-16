#!/bin/zsh
# usage: p8run.zsh NAME   -> runs the block after "<!-- p8:NAME -->" of the committed procedure, in a zsh that has the session variables
# usage: p8run.zsh --save NAME FILE -> writes that block to FILE (appendix files)
# ENVF and DOC are the paths of the 2026-10-31 refresh: change both for a new one. Record each block's result in RECORD.md by hand.
ENVF=~/.route_sync/work/fp1031/phase8/env.sh
DOC=~/.route_sync/work/fp1031/port-wt/docs/frogpilot_refresh_20261031/PHASE8_PROCEDURE.md
extract() { awk -v m="<!-- p8:$1 -->" '$0==m{f=1;next} f&&!g&&/^```/{g=1;next} f&&g&&/^```$/{exit} f&&g{print}' "$DOC"; }
if [[ "$1" == --save ]]; then extract "$2" > "$3"; wc -l "$3"; exit 0; fi
[[ -f $ENVF ]] && source $ENVF && source "$REPO/.venv/bin/activate"
blk=$(mktemp -t p8blk) ; extract "$1" > $blk
[[ -s $blk ]] || { echo "no block p8:$1"; exit 99; }
source $blk; rc=$?
{ for v in DEV REPO PORT TIP REVIEWED MAPD_PROVEN TILES_MAIN TILES_SI MIN_FREE P8 CAND_MAPD OLD_MAPD TILES_DIGEST TILES_COUNT TILES_BYTES UNIT UNIT_CGROUP DEVICE_HEAD ORIGIN_NEW OLD_SHA CAND TARGET_SHA TARGET_BRANCH TARGET_MODE; do [[ -n "${(P)v}" ]] && print -r -- "export $v=${(qq)${(P)v}}"; done; true; } > $ENVF.new; mv $ENVF.new $ENVF
rm -f $blk
echo "[p8run $1 rc=$rc]"
exit $rc
