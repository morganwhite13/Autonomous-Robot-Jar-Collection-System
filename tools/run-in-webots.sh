#!/usr/bin/env bash
# Runs v1 and/or v2 in Webots without a window and compares them.
#
#   WEBOTS_HOME=/usr/local/webots tools/run-in-webots.sh            # both versions
#   WEBOTS_HOME=/usr/local/webots tools/run-in-webots.sh v2         # just one
#   WEBOTS_HOME=/usr/local/webots tools/run-in-webots.sh v1 v2 v2-planned
#                       # v2-planned: v2 with Config.USE_PLANNED_ROUTES switched on (A* routes)
#   tools/run-in-webots.sh --dry-run                                # build everything, don't launch
#   WEBOTS_HOME=/usr/local/webots tools/run-in-webots.sh --watch v2 # watch it in real time
#
# Options: --limit SECONDS (simulated-time limit per run, default 3600)
#          --watch         open the Webots window and run at real-time speed, e.g. to record a
#                          demo video (needs a display; same observer, same results)
#          --seed N        shuffle the jars and nudge the robot's start pose (tools/WorldShuffle.java);
#                          0, the default, is the world as it is
#          --out DIR       where to put the runs (default build/webots-run); docs/ is only updated
#                          for the default folder and seed 0
#
# Each run gets a temporary copy of the project under build/webots-run/<version>/. The copy's
# world adds an invisible "observer" supervisor (tools/webots/Observer.java) that records the
# robot's path and when each jar reaches the drop-off, then ends the simulation. The real
# worlds and controllers are not modified. Afterwards the script writes
# build/webots-run/report.md and build/webots-run/paths.svg; with both versions it also copies
# them to docs/webots-results.md and docs/images/webots-paths.svg.
#
# Linux needs xvfb-run (a virtual display) unless a display is available. macOS: set
# WEBOTS_HOME=/Applications/Webots.app.
set -euo pipefail
cd "$(dirname "$0")/.."

LIMIT=3600
DRY_RUN=0
WATCH=0
SEED=0
RUN_ROOT="build/webots-run"
VERSIONS=()
while [ $# -gt 0 ]; do
  case "$1" in
    --limit) LIMIT="$2"; shift 2 ;;
    --seed) SEED="$2"; shift 2 ;;
    --out) RUN_ROOT="$2"; shift 2 ;;
    --dry-run) DRY_RUN=1; shift ;;
    --watch) WATCH=1; shift ;;
    v1|v2|v2-planned) VERSIONS+=("$1"); shift ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done
[ ${#VERSIONS[@]} -eq 0 ] && VERSIONS=(v1 v2)

# --- locate Webots ------------------------------------------------------------------------
CONTROLLER_JAR=""
WEBOTS_BIN=""
for home in "${WEBOTS_HOME:-}" /usr/local/webots /Applications/Webots.app; do
  [ -n "$home" ] || continue
  for jar in "$home/lib/controller/java/Controller.jar" "$home/Contents/lib/controller/java/Controller.jar"; do
    [ -f "$jar" ] && CONTROLLER_JAR="$jar"
  done
  for bin in "$home/webots" "$home/Contents/MacOS/webots"; do
    [ -x "$bin" ] && WEBOTS_BIN="$bin"
  done
  [ -n "$CONTROLLER_JAR" ] && [ -n "$WEBOTS_BIN" ] && break
  CONTROLLER_JAR=""
  WEBOTS_BIN=""
done
if [ -z "$CONTROLLER_JAR" ]; then
  if [ "$DRY_RUN" = 1 ]; then
    echo "==> Webots not found; dry run compiles controllers against the API stubs"
    rm -rf build/stubs
    find tools/webots-stubs -name '*.java' -exec javac --release 11 -d build/stubs {} +
    CONTROLLER_JAR="build/stubs"
    WEBOTS_BIN="webots"
  else
    echo "Webots not found. Set WEBOTS_HOME to your Webots install (the folder holding lib/controller/java/Controller.jar)." >&2
    exit 1
  fi
fi

CONTROLLER_JAR="$(cd "$(dirname "$CONTROLLER_JAR")" && pwd)/$(basename "$CONTROLLER_JAR")"

LAUNCH=("$WEBOTS_BIN")
VIEW=(--mode=fast --no-rendering --minimize)
if [ "$WATCH" = 1 ]; then
  VIEW=(--mode=realtime)
elif [ "$(uname)" = Linux ] && [ -z "${DISPLAY:-}" ]; then
  LAUNCH=(xvfb-run -a -s "-screen 0 1280x1024x24" "$WEBOTS_BIN")
fi

mkdir -p "$RUN_ROOT"
TOOLS="$RUN_ROOT/tools"
rm -rf "$TOOLS" && mkdir -p "$TOOLS"
javac --release 11 -nowarn -d "$TOOLS" controllers/JarCollectorV2/{Vec2,Angles,Routes}.java \
  tools/WorldFile.java controllers/JarCollectorV2/RoutePlanner.java tools/MapRenderer.java tools/WebotsReport.java \
  tools/WorldShuffle.java

for version in "${VERSIONS[@]}"; do
  case "$version" in
    v1) controller=ProjectController3; world=ProjectWorld2025.wbt ;;
    v2|v2-planned) controller=JarCollectorV2; world=ProjectWorld2025_v2.wbt ;;
  esac
  dir="$RUN_ROOT/$version"
  echo "==> Preparing $version in $dir"
  rm -rf "$dir"
  mkdir -p "$dir/worlds" "$dir/controllers/Observer" "$dir/protos"
  cp protos/*.proto "$dir/protos/"
  cp -R "controllers/$controller" "$dir/controllers/"
  if [ "$version" = v2-planned ]; then
    config="$dir/controllers/$controller/Config.java"
    sed 's/USE_PLANNED_ROUTES = false;/USE_PLANNED_ROUTES = true;/' "$config" > "$config.tmp"
    mv "$config.tmp" "$config"
    grep -q 'USE_PLANNED_ROUTES = true;' "$config" || { echo "could not switch on planned routes" >&2; exit 1; }
  fi
  cp tools/webots/Observer.java tools/webots/DeliveryTracker.java "$dir/controllers/Observer/"
  out_abs="$(cd "$dir" && pwd)"

  # Name the robot and jars so the observer can find them, and add the observer node.
  awk -v out="$out_abs" -v limit="$LIMIT" '
    /^Pioneer3dx \{/ { sub(/^Pioneer3dx/, "DEF ROBOT Pioneer3dx") }
    /^HoneyJar \{/ { sub(/^HoneyJar/, "DEF JAR" jars++ " HoneyJar") }
    { print }
    END {
      print "Robot {"
      print "  name \"observer\""
      print "  controller \"Observer\""
      print "  controllerArgs [ \"" out "\" \"" limit "\" ]"
      print "  supervisor TRUE"
      print "}"
    }' "worlds/$world" > "$dir/worlds/$world"
  if [ "$SEED" != 0 ]; then
    java -cp "$TOOLS" WorldShuffle "$dir/worlds/$world" "$SEED" | sed 's/^/    /'
  fi

  # Compile the controllers up front (what Webots' Build button does) so a compile error
  # shows up here rather than as a silent controller crash.
  for c in "$controller" Observer; do
    (cd "$dir/controllers/$c" && javac -nowarn -encoding UTF-8 -cp "$CONTROLLER_JAR:." ./*.java)
  done

  cmd=("${LAUNCH[@]}" --batch "${VIEW[@]}" --stdout --stderr "$out_abs/worlds/$world")
  if [ "$DRY_RUN" = 1 ]; then
    echo "    would run: ${cmd[*]}"
    continue
  fi
  echo "==> Running $version (simulated-time limit ${LIMIT} s); log: $dir/webots.log"
  start=$(date +%s)
  "${cmd[@]}" > "$dir/webots.log" 2>&1 || echo "    Webots exited with status $?"
  echo "    finished in $(( $(date +%s) - start )) s of wall time"
  if [ -f "$dir/result.properties" ]; then
    sed 's/^/    /' "$dir/result.properties"
  else
    echo "    no result.properties: the observer did not finish (see $dir/webots.log)"
  fi
done

[ "$DRY_RUN" = 1 ] && exit 0

echo "==> Writing report"
java -cp "$TOOLS" WebotsReport "$RUN_ROOT" "${VERSIONS[@]}" > "$RUN_ROOT/report.md"
paths=()
for version in "${VERSIONS[@]}"; do
  [ -f "$RUN_ROOT/$version/trajectory.csv" ] && paths+=(--path "$version=$RUN_ROOT/$version/trajectory.csv")
done
if [ ${#paths[@]} -gt 0 ]; then
  map_world=worlds/ProjectWorld2025.wbt
  if [ "$SEED" != 0 ]; then # draw the shuffled jars
    map_world=$(find "$RUN_ROOT/${VERSIONS[0]}/worlds" -name '*.wbt' | head -1)
  fi
  java -cp "$TOOLS" MapRenderer "$map_world" "$RUN_ROOT/paths.svg" "${paths[@]}"
fi
cat "$RUN_ROOT/report.md"

ran() { # version
  local v
  for v in "${VERSIONS[@]}"; do [ "$v" = "$1" ] && return 0; done
  return 1
}
if ran v1 && ran v2 && [ -f "$RUN_ROOT/paths.svg" ] && [ "$SEED" = 0 ] \
    && [ "$RUN_ROOT" = build/webots-run ]; then
  {
    echo "# Webots results"
    echo
    echo "Both worlds run headless in Webots by \`tools/run-in-webots.sh\` on $(date -u +%Y-%m-%d)," \
      "each with a ${LIMIT} s simulated-time limit. An observer supervisor judges both runs the same way:" \
      "a jar counts as delivered once it rests in the drop-off lane for 5 s."
    echo
    cat "$RUN_ROOT/report.md"
    echo
    echo "![Paths driven in Webots: v1 orange, v2 blue, v2-planned (if run) green](images/webots-paths.svg)"
  } > docs/webots-results.md
  cp "$RUN_ROOT/paths.svg" docs/images/webots-paths.svg
  echo "==> Updated docs/webots-results.md and docs/images/webots-paths.svg"
fi
