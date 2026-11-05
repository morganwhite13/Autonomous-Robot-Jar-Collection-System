#!/usr/bin/env bash
# Turns the frames saved by tools/run-in-webots.sh --frames into an animated GIF.
#
#   tools/make-demo-gif.sh build/webots-run/v2/frames docs/images/v2-webots.gif [frames-per-second]
#
# Needs ffmpeg. Each frame is one --frames interval of simulated time, so with --frames 3 and the
# default 10 frames per second the GIF plays at 30x speed. WIDTH sets its width (default 560).
set -euo pipefail
FRAMES_DIR=$1
OUT=$2
FPS=${3:-10}
WIDTH=${WIDTH:-560}
command -v ffmpeg > /dev/null || { echo "ffmpeg not found" >&2; exit 1; }
ffmpeg -loglevel error -y -framerate "$FPS" -i "$FRAMES_DIR/frame%05d.png" \
  -vf "scale=$WIDTH:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=48:stats_mode=diff[p];[b][p]paletteuse=dither=bayer:bayer_scale=4:diff_mode=rectangle" \
  -loop 0 "$OUT"
echo "Wrote $OUT ($(du -h "$OUT" | cut -f1))"
