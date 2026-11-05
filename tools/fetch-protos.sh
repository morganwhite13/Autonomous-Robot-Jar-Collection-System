#!/usr/bin/env bash
# UNTESTED helper: downloads the Webots R2021b (NUE) Pioneer 3-DX protos into protos/ and
# updates their headers so Webots R2025a can load them. See protos/README.md for why these files
# are needed and why they are not committed. Prefer copying your original protos if you have them.
set -euo pipefail
cd "$(dirname "$0")/../protos"

BASE=https://raw.githubusercontent.com/cyberbotics/webots/R2021b/projects/robots/adept/pioneer3/protos
for name in Pioneer3dx Pioneer3Gripper Pioneer3DistanceSensor; do
  if [ -f "$name.proto" ]; then
    echo "protos/$name.proto already exists, leaving it alone"
    continue
  fi
  echo "Downloading $name.proto (Webots R2021b)"
  curl -fsSL --retry 3 "$BASE/$name.proto" -o "$name.proto.tmp"
  # R2021b header -> R2025a header.
  sed '1s/.*/#VRML_SIM R2025a utf8/' "$name.proto.tmp" > "$name.proto"
  rm "$name.proto.tmp"
done

# Since R2023a a proto must declare the protos it uses.
if ! grep -q 'EXTERNPROTO "Pioneer3DistanceSensor.proto"' Pioneer3dx.proto; then
  sed -i.bak '1a\
EXTERNPROTO "Pioneer3DistanceSensor.proto"' Pioneer3dx.proto && rm -f Pioneer3dx.proto.bak
fi

echo "Done. These files are Cyberbotics assets (\"Licensed for use only with Webots\")."
