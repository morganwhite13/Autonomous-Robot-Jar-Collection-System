#!/usr/bin/env bash
# Builds both controllers, runs the unit tests, and regenerates the generated docs
# (docs/benchmark-results.md, docs/images/mission-map.svg and docs/images/before-after.svg).
#
# Needs a JDK (11+) and curl. Webots is optional: if WEBOTS_HOME points at a Webots install the
# controllers are compiled against its real Controller.jar, otherwise against the compile-only
# stubs in tools/webots-stubs.
set -euo pipefail
cd "$(dirname "$0")/.."

JUNIT_VERSION=1.10.2
BUILD=build
JUNIT_JAR="$BUILD/lib/junit-platform-console-standalone-$JUNIT_VERSION.jar"
JAVAC=(javac --release 11 -encoding UTF-8)

rm -rf "$BUILD/v1" "$BUILD/v2" "$BUILD/test" "$BUILD/stubs"
mkdir -p "$BUILD/lib"

if [ ! -f "$JUNIT_JAR" ]; then
  echo "==> Downloading JUnit $JUNIT_VERSION"
  curl -fsSL --retry 5 --retry-delay 3 --retry-all-errors -o "$JUNIT_JAR" \
    "https://repo1.maven.org/maven2/org/junit/platform/junit-platform-console-standalone/$JUNIT_VERSION/junit-platform-console-standalone-$JUNIT_VERSION.jar"
fi

if [ -n "${WEBOTS_HOME:-}" ] && [ -f "$WEBOTS_HOME/lib/controller/java/Controller.jar" ]; then
  WEBOTS_CP="$WEBOTS_HOME/lib/controller/java/Controller.jar"
  echo "==> Using the real Webots API from $WEBOTS_HOME"
else
  echo "==> WEBOTS_HOME not set: using compile-only stubs in tools/webots-stubs"
  "${JAVAC[@]}" -d "$BUILD/stubs" $(find tools/webots-stubs -name '*.java')
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
java -jar "$JUNIT_JAR" execute --class-path "$BUILD/test" --scan-class-path \
  --disable-banner --details=tree --fail-if-no-tests

echo "==> Regenerating docs/benchmark-results.md and the figures in docs/images"
java -cp "$BUILD/test" BeforeAfterBenchmark worlds/ProjectWorld2025.wbt > docs/benchmark-results.md
java -cp "$BUILD/test" MapRenderer worlds/ProjectWorld2025.wbt docs/images/mission-map.svg
java -cp "$BUILD/test" BeforeAfterFigure docs/images/before-after.svg

echo "==> All good"
