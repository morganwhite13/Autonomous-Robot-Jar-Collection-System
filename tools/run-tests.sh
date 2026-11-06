#!/usr/bin/env bash
# Builds both controllers, runs the unit tests, and regenerates the generated docs
# (docs/benchmark-results.md, docs/route-planner.md and the figures in docs/images).
#
# Needs a JDK (11+) and curl. Webots is optional: if WEBOTS_HOME points at a Webots install the
# controllers are compiled against its real Controller.jar, otherwise against the compile-only
# stubs in tools/webots-stubs.
#
#   tools/run-tests.sh              build, test, regenerate docs
#   tools/run-tests.sh --coverage   also measure line coverage of the v2 controller (JaCoCo);
#                                   HTML report in build/coverage/index.html
set -euo pipefail
cd "$(dirname "$0")/.."

COVERAGE=0
[ "${1:-}" = "--coverage" ] && COVERAGE=1

JUNIT_VERSION=1.10.2
BUILD=build
JUNIT_JAR="$BUILD/lib/junit-platform-console-standalone-$JUNIT_VERSION.jar"
JAVAC=(javac --release 11 -encoding UTF-8)
JACOCO_VERSION=0.8.12
MAVEN=https://repo1.maven.org/maven2

rm -rf "$BUILD/v1" "$BUILD/v2" "$BUILD/test" "$BUILD/stubs" "$BUILD/coverage" "$BUILD/jacoco.exec"
mkdir -p "$BUILD/lib"

download() { # url target
  if [ ! -f "$2" ]; then
    echo "==> Downloading $(basename "$2")"
    curl -fsSL --retry 8 --retry-delay 10 --retry-all-errors -o "$2" "$1"
  fi
}
download "$MAVEN/org/junit/platform/junit-platform-console-standalone/$JUNIT_VERSION/junit-platform-console-standalone-$JUNIT_VERSION.jar" "$JUNIT_JAR"
AGENT=()
if [ "$COVERAGE" = 1 ]; then
  download "$MAVEN/org/jacoco/org.jacoco.agent/$JACOCO_VERSION/org.jacoco.agent-$JACOCO_VERSION-runtime.jar" "$BUILD/lib/jacocoagent.jar"
  download "$MAVEN/org/jacoco/org.jacoco.cli/$JACOCO_VERSION/org.jacoco.cli-$JACOCO_VERSION-nodeps.jar" "$BUILD/lib/jacococli.jar"
  AGENT=("-javaagent:$BUILD/lib/jacocoagent.jar=destfile=$BUILD/jacoco.exec")
fi

if [ -n "${WEBOTS_HOME:-}" ] && [ -f "$WEBOTS_HOME/lib/controller/java/Controller.jar" ]; then
  WEBOTS_CP="$WEBOTS_HOME/lib/controller/java/Controller.jar"
  echo "==> Using the real Webots API from $WEBOTS_HOME"
else
  echo "==> WEBOTS_HOME not set: using compile-only stubs in tools/webots-stubs"
  find tools/webots-stubs -name '*.java' -exec "${JAVAC[@]}" -d "$BUILD/stubs" {} +
  WEBOTS_CP="$BUILD/stubs"
fi

echo "==> Compiling v1 controller (original, unchanged)"
"${JAVAC[@]}" -nowarn -cp "$WEBOTS_CP" -d "$BUILD/v1" controllers/ProjectController3/ProjectController3.java

echo "==> Compiling v2 controller (warnings are errors)"
"${JAVAC[@]}" -Xlint:all -Werror -cp "$WEBOTS_CP" -d "$BUILD/v2" controllers/JarCollectorV2/*.java

echo "==> Compiling tests and tools"
"${JAVAC[@]}" -Xlint:all -Werror -cp "$JUNIT_JAR:$WEBOTS_CP" -d "$BUILD/test" \
  controllers/JarCollectorV2/*.java tools/*.java tools/webots/*.java tests/*.java

echo "==> Running tests"
java ${AGENT[@]+"${AGENT[@]}"} -jar "$JUNIT_JAR" execute --class-path "$BUILD/test" --scan-class-path \
  --disable-banner --details=tree --fail-if-no-tests

if [ "$COVERAGE" = 1 ]; then
  echo "==> Line coverage of the v2 controller (classes the tests ran, from $BUILD/test)"
  mkdir -p "$BUILD/coverage-classes"
  for src in controllers/JarCollectorV2/*.java; do
    name="$(basename "$src" .java)"
    cp "$BUILD/test/$name.class" "$BUILD/test/$name"\$*.class "$BUILD/coverage-classes/" 2>/dev/null || true
  done
  java -jar "$BUILD/lib/jacococli.jar" report "$BUILD/jacoco.exec" --quiet \
    --classfiles "$BUILD/coverage-classes" --sourcefiles controllers/JarCollectorV2 \
    --html "$BUILD/coverage" --csv "$BUILD/coverage.csv"
  # CSV columns: GROUP,PACKAGE,CLASS,...,LINE_MISSED(8),LINE_COVERED(9),...
  awk -F, 'NR > 1 {
      split($3, outer, "."); cls = outer[1]; sub(/\$.*/, "", cls)
      missed[cls] += $8; covered[cls] += $9
    }
    END {
      for (c in covered) {
        total = missed[c] + covered[c]
        if (total > 0) printf "    %-16s %4d/%-4d lines  %5.1f%%\n", c, covered[c], total, 100 * covered[c] / total
        all_m += missed[c]; all_c += covered[c]
        if (c != "WebotsPlatform" && c != "JarCollectorV2") { lm += missed[c]; lc += covered[c] }
      }
      printf "    %-16s %4d/%-4d lines  %5.1f%%\n", "ALL", all_c, all_m + all_c, 100 * all_c / (all_m + all_c)
      printf "    %-16s %4d/%-4d lines  %5.1f%%  (excluding the two classes that need Webots)\n", "LOGIC", lc, lm + lc, 100 * lc / (lm + lc)
    }' "$BUILD/coverage.csv" | sort
  echo "    HTML report: $BUILD/coverage/index.html"
fi

echo "==> Regenerating docs/benchmark-results.md, docs/route-planner.md and the figures in docs/images"
java -cp "$BUILD/test" BeforeAfterBenchmark worlds/ProjectWorld2025.wbt > docs/benchmark-results.md
java -cp "$BUILD/test" MapRenderer worlds/ProjectWorld2025.wbt docs/images/mission-map.svg
java -cp "$BUILD/test" BeforeAfterFigure docs/images/before-after.svg
java -cp "$BUILD/test" BeforeAfterFigure docs/images/before-after-dark.svg --dark
java -cp "$BUILD/test" MissionReplay worlds/ProjectWorld2025.wbt docs/images/mission-replay.svg
java -cp "$BUILD/test" RoutePlannerReport worlds/ProjectWorld2025.wbt docs/route-planner.md docs/images/route-planner.svg

echo "==> All good"
