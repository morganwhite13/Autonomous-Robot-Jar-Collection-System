#!/usr/bin/env bash
# Robustness sweep: runs each version on several shuffled worlds (jars placed at random in the
# jar bay, start pose nudged; see tools/WorldShuffle.java) and reports how often each delivers
# every jar.
#
#   WEBOTS_HOME=/usr/local/webots tools/webots-sweep.sh --seeds 1-10 --jobs 3 v1 v2
#
# Options: --seeds A-B   seeds to run (default 1-10; seed 0 is the unshuffled world)
#          --jobs N      worlds to run at once (default 2; each Webots uses about one core)
#          --limit S     simulated-time limit per run (default 1800)
#          --publish     also copy the report to docs/webots-sweep.md
#
# Each seed's runs go to build/webots-sweep/seed-<n>/ (as tools/run-in-webots.sh lays them out)
# and the summary to build/webots-sweep/report.md.
set -euo pipefail
cd "$(dirname "$0")/.."

SEEDS=1-10
JOBS=2
LIMIT=1800
PUBLISH=0
VERSIONS=()
while [ $# -gt 0 ]; do
  case "$1" in
    --seeds) SEEDS="$2"; shift 2 ;;
    --jobs) JOBS="$2"; shift 2 ;;
    --limit) LIMIT="$2"; shift 2 ;;
    --publish) PUBLISH=1; shift ;;
    v1|v2|v2-planned) VERSIONS+=("$1"); shift ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done
[ ${#VERSIONS[@]} -eq 0 ] && VERSIONS=(v1 v2)
first=${SEEDS%-*}
last=${SEEDS#*-}

OUT=build/webots-sweep
mkdir -p "$OUT"
echo "==> Seeds $first-$last, versions ${VERSIONS[*]}, $JOBS at a time, limit $LIMIT s"
export LIMIT OUT
seq "$first" "$last" | xargs -P "$JOBS" -I{} sh -c \
  'tools/run-in-webots.sh --seed {} --out "$OUT/seed-{}" --limit "$LIMIT" '"${VERSIONS[*]}"' \
     > "$OUT/seed-{}.log" 2>&1 && echo "    seed {} done" || echo "    seed {} FAILED (see $OUT/seed-{}.log)"'

echo "==> Writing report"
TOOLS="$OUT/tools"
rm -rf "$TOOLS" && mkdir -p "$TOOLS"
javac --release 11 -nowarn -d "$TOOLS" controllers/JarCollectorV2/{Vec2,Angles}.java \
  tools/WebotsReport.java tools/WebotsSweepReport.java
java -cp "$TOOLS" WebotsSweepReport "$OUT" "$first" "$last" "$LIMIT" "${VERSIONS[@]}" > "$OUT/report.md"
cat "$OUT/report.md"
if [ "$PUBLISH" = 1 ]; then
  cp "$OUT/report.md" docs/webots-sweep.md
  echo "==> Updated docs/webots-sweep.md"
fi
