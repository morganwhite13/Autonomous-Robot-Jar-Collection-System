#!/usr/bin/env bash
# Shows how the committed protos/*.proto files were made: download the Webots R2021b (NUE)
# Pioneer 3-DX protos and declare the one proto they use. The R2021b header stays: it is what
# makes Webots R2025a convert the old cylinder and camera conventions (see protos/README.md).
# Existing files are left alone, so delete protos/*.proto first to regenerate them.
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
  # Keep the header: changing it to R2025a switches off Webots' conversion of the old
  # conventions, and the wheels and camera come out rotated.
  head -1 "$name.proto.tmp" | grep -q '^#VRML_SIM R2021b' || { echo "unexpected header in $name.proto" >&2; exit 1; }
  mv "$name.proto.tmp" "$name.proto"
done

# Since R2023a a proto must declare the protos it uses.
if ! grep -q 'EXTERNPROTO "Pioneer3DistanceSensor.proto"' Pioneer3dx.proto; then
  sed -i.bak '1a\
EXTERNPROTO "Pioneer3DistanceSensor.proto"' Pioneer3dx.proto && rm -f Pioneer3dx.proto.bak
fi

echo "Done. These files are Cyberbotics assets (\"Licensed for use only with Webots\")."
