#!/usr/bin/env bash
# Compiles both Java controllers in place so Webots can run them.
#
# Webots does not compile Java controllers by itself: it runs the .class it finds in
# controllers/<name>/, and with only a .java file it reports "Could not find controller file".
# Run this once after cloning (and after editing a controller), or press Build in Webots'
# text editor with the controller's main .java file open.
#
#   WEBOTS_HOME=/usr/local/webots tools/build-controllers.sh      # macOS: /Applications/Webots.app
set -euo pipefail
cd "$(dirname "$0")/.."

JAR=""
for home in "${WEBOTS_HOME:-}" /usr/local/webots /Applications/Webots.app; do
  [ -n "$home" ] || continue
  for candidate in "$home/lib/controller/java/Controller.jar" "$home/Contents/lib/controller/java/Controller.jar"; do
    if [ -f "$candidate" ]; then
      JAR="$candidate"
      break 2
    fi
  done
done
if [ -z "$JAR" ]; then
  echo "Webots not found. Set WEBOTS_HOME to your Webots install (the folder holding lib/controller/java/Controller.jar)." >&2
  exit 1
fi
JAR="$(cd "$(dirname "$JAR")" && pwd)/$(basename "$JAR")"

for controller in ProjectController3 JarCollectorV2; do
  echo "==> Building controllers/$controller"
  # Same command as Webots' Build button: compile every source in the folder.
  (cd "controllers/$controller" && javac -nowarn -encoding UTF-8 -cp "$JAR:." ./*.java)
done
echo "==> Done. Open worlds/ProjectWorld2025.wbt (v1) or worlds/ProjectWorld2025_v2.wbt (v2) in Webots."
