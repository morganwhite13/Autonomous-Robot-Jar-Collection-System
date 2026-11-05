# Webots results

Both worlds run headless in Webots by `tools/run-in-webots.sh` on 2026-11-05, each with a 2400 s simulated-time limit. An observer supervisor judges both runs the same way: a jar counts as delivered once it rests in the drop-off lane for 5 s.

| | v1 | v2 | v2-planned |
|---|---|---|---|
| Jars delivered | 0/5 | 5/5 | 5/5 |
| Jars the controller says it delivered | 3 | 5 | 5 |
| Mission time (last jar delivered) | did not finish (stopped at 2400 s (40.0 min)) | 879 s (14.7 min) | 788 s (13.1 min) |
| Jar delivery times (s) | - | 189, 353, 527, 733, 879 | 177, 321, 474, 655, 788 |
| Stuck in one place from | 745 s to the end | - | - |
| Distance driven | 199 m | 276 m | 247 m |
| Jars knocked out of the drop-off lane afterwards | 0 | 0 | 0 |
| Approach aborted | 6 | 0 | 0 |
| Full scan saw no jar | 0 | 0 | 0 |
| Whole search failed | 1 | 0 | 0 |
| Drive stalled or timed out (v2 detects these; v1 cannot) | 0 | 0 | 0 |

![Paths driven in Webots: v1 orange, v2 blue, v2-planned (if run) green](images/webots-paths.svg)
