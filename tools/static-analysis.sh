#!/usr/bin/env bash
# Runs SpotBugs (maximum effort, every priority) on both controllers.
# Fails if v2 has any finding; v1's findings are printed for comparison only.
#
# Needs a JDK and curl. Webots is optional (WEBOTS_HOME); without it the controllers are
# compiled against the API stubs in tools/webots-stubs.
set -euo pipefail
cd "$(dirname "$0")/.."

VERSION=4.8.6
BUILD=build
SPOTBUGS_DIR="$BUILD/lib/spotbugs-$VERSION"
OUT="$BUILD/spotbugs"

if [ ! -f "$SPOTBUGS_DIR/lib/spotbugs.jar" ]; then
  echo "==> Downloading SpotBugs $VERSION"
  mkdir -p "$BUILD/lib"
  curl -fsSL --retry 8 --retry-delay 10 --retry-all-errors -o "$BUILD/lib/spotbugs.tgz" \
    "https://github.com/spotbugs/spotbugs/releases/download/$VERSION/spotbugs-$VERSION.tgz"
  tar xzf "$BUILD/lib/spotbugs.tgz" -C "$BUILD/lib"
  rm "$BUILD/lib/spotbugs.tgz"
fi

rm -rf "$OUT"
mkdir -p "$OUT/v1" "$OUT/v2"
if [ -n "${WEBOTS_HOME:-}" ] && [ -f "$WEBOTS_HOME/lib/controller/java/Controller.jar" ]; then
  API="$WEBOTS_HOME/lib/controller/java/Controller.jar"
else
  API="$OUT/stubs"
  find tools/webots-stubs -name '*.java' -exec javac --release 11 -d "$API" {} +
fi
javac --release 11 -nowarn -cp "$API" -d "$OUT/v1" controllers/ProjectController3/ProjectController3.java
javac --release 11 -nowarn -cp "$API" -d "$OUT/v2" controllers/JarCollectorV2/*.java

findings() { # version
  java -jar "$SPOTBUGS_DIR/lib/spotbugs.jar" -textui -effort:max -low -auxclasspath "$API" \
    "$OUT/$1" > "$OUT/$1.txt" 2> "$OUT/$1.err" || true
  grep -c . "$OUT/$1.txt" || true
}

v1=$(findings v1)
v2=$(findings v2)
echo "==> SpotBugs, v1 (original): $v1 findings, for comparison"
sed 's/^/    /' "$OUT/v1.txt"
echo "==> SpotBugs, v2: $v2 findings"
sed 's/^/    /' "$OUT/v2.txt"
if [ "$v2" != 0 ]; then
  echo "v2 must have no SpotBugs findings" >&2
  exit 1
fi
