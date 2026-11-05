# Before and after

This repository holds two versions of the same robot controller, side by side:

| | v1 (before) | v2 (after) |
|---|---|---|
| Controller | [`controllers/ProjectController3/`](../controllers/ProjectController3/ProjectController3.java) | [`controllers/JarCollectorV2/`](../controllers/JarCollectorV2/) |
| World | [`worlds/ProjectWorld2025.wbt`](../worlds/ProjectWorld2025.wbt) | [`worlds/ProjectWorld2025_v2.wbt`](../worlds/ProjectWorld2025_v2.wbt) |
| Status | Original code, **byte-for-byte unchanged** (only moved into a Webots project layout) | Same algorithm, restructured, with the bugs below fixed |

The two world files are identical except for one line: `controller "ProjectController3"` versus `controller "JarCollectorV2"`. v2 keeps every waypoint, gripper position and timing, approach speed, steering rule and vision threshold from v1 (`RoutesTest` and `VisionTest` check this against the original values). The changes are in *how* the robot moves between those points and what it does when something goes wrong.

Line numbers below refer to [`controllers/ProjectController3/ProjectController3.java`](../controllers/ProjectController3/ProjectController3.java).

---

## How this was checked, and what still needs checking

**Done:**
- Both controllers compile against the real Webots R2025a `Controller.jar`. CI compiles them against API stubs whose signatures were copied from that jar.
- 46 JUnit tests. Most run v2's real navigation, search and mission classes on a kinematic model of the Pioneer 3-DX (wheel radius 9.75 cm, axle track 33 cm, 32 ms steps, synthetic 64x64 camera).
- The v1 methods being compared were ported line for line into [`tests/LegacyV1.java`](../tests/LegacyV1.java) and measured on the same model ([`docs/benchmark-results.md`](benchmark-results.md)). [`LegacyBehaviourTest`](../tests/LegacyBehaviourTest.java) pins each v1 behaviour described here, so the claims fail loudly if they stop being true.
- Route geometry was checked against the obstacles parsed from the `.wbt` file.

**Not done yet:** neither version has been run in Webots as part of this change. Webots could not be launched in the environment this was written in, and the robot protos are not in the repository (see [`protos/README.md`](../protos/README.md)). The kinematic model has no inertia, wheel slip or box physics. Running both worlds in Webots and comparing v2's end-of-run mission report with v1's run time is the next step. Every behaviour change can be switched back to v1's behaviour in [`Config.java`](../controllers/JarCollectorV2/Config.java) (see the last section).

---

## Measured on the kinematic model

| Check | v1 (before) | v2 (after) |
|---|---|---|
| Turn-in-place finishes (6,240 start/target pairs) | 91.8% | **100%** |
| Turn-in-place mean steps when it finishes | 16.7 | 22.9 (slower near the target, on purpose) |
| Rotation actually covered by the "360°" jar scan | **271°** | 371° |
| Scan finds a jar 1 m away, 45° right of the starting heading | no (blind spot) | yes |
| 530 cm leg started 2.9° off (inside the ±3° turn tolerance): distance from the waypoint at the end | **26.8 cm** | 1.1 cm |
| Drive blocked by an immovable obstacle | never returns | stall detected after 3.8 s |
| Longest approach before the time limit aborts it | 59 cm | 176 cm, and aborts early only if not moving |
| Route after a failed search for jars 2-5 | 41 m, first leg through **4 static wooden boxes** | 0.8 m |
| Mission when no jar can be found | retries forever | stops after 3 failed searches with a report |
| Whole mission | - | 5/5 jars, 815 s simulated, 0 stalls |

The full generated table is in [`benchmark-results.md`](benchmark-results.md). Regenerate it with `tools/run-tests.sh`.

---

## Bugs fixed

### 1. A failed search sent the robot straight across the arena (serious)
**v1:** `main` picks the route to the search point with `if (jarsToGo == 5)` (line 130). Only jar 1 uses the short route. For jars 2-5 it uses `navigateFromDropoffToSearch()`, which assumes the robot is at the drop-off. After a failed search (lines 144-148) the robot is still at the search point, so the route's first leg (line 588) is a straight 17.8 m line from the jar bay to the drop-off. That line passes through four wooden boxes, which have no mass in the world file and so cannot be pushed. Combined with bug 3 below, the robot could push against one forever.

**v2:** [`Mission`](../controllers/JarCollectorV2/Mission.java) tracks whether the robot is at the drop-off and takes the 0.8 m hop back to the search point after a failed search. The red dotted line on the [mission map](images/mission-map.svg) shows the old route.

### 2. Endless retries
**v1:** `while (jarsToGo > 0)` (line 126) has no attempt limit, so a jar that cannot be found makes the program run forever.
**v2:** Stops after `MAX_CONSECUTIVE_FAILED_SEARCHES` (3) failures in a row and prints what it achieved.

### 3. Drives had no timeout or stall detection
**v1:** `moveHardcodedAhead` (lines 709-715) loops until the robot is far enough from where it started. If the robot is blocked, that never happens.
**v2:** [`Navigator.driveTo`](../controllers/JarCollectorV2/Navigator.java) checks progress every 40 steps. If the robot moved less than 2 cm, it reports a stall and continues with the next waypoint. Each drive also has a hard timeout. Both are counted in the mission report.

### 4. Turns could bounce around the target heading
**v1:** `makeHardcodedTurn` (lines 722-765) always turns at full speed, about 5.4° per 32 ms step, and stops only when the integer heading lands strictly inside `finalAngle ± 3` (line 748). That window is 5 values wide, so a step can jump over it, reverse, and jump back. On the model, 8.2% of start/target pairs never finished. An independent check with a different heading-only model ([PR #1](https://github.com/morganwhite13/Autonomous-Robot-Jar-Collection-System/pull/1)) found about the same rate: 135 of 1,296 turns never settled. The check also doesn't wrap around 0/360: a target of 0° only accepts 0-2°, and 359° only accepts 357-359°. It also always turns for one step before checking, even when already aligned.
**v2:** Uses a seam-safe heading error, full speed down to 30° from the target, then proportional slow-down. It checks before moving and has a timeout. It finished 100% of pairs, at about 0.2 s extra per turn.

### 5. Straight legs drifted off line
**v1:** Drives open-loop. A turn that ends 2.9° off (allowed by the ±3° tolerance) puts the robot 26.8 cm from the waypoint at the end of the 530 cm leg. Box pushing adds more error.
**v2:** Steers towards the waypoint while driving: a proportional correction capped at 20% of wheel speed, switched off in the last 20 cm. It ends 1.1 cm away.

### 6. The "360°" scan covered about 270°
**v1:** `rotateAndFindGreen` rotates for a fixed 200 steps at `MAX_SPEED / 4` (lines 206-212). With the Pioneer's wheel geometry and the default 32 ms step that is about 271°. Since the robot always turns left, a jar slightly right of the starting heading is never in view.
**v2:** Adds up the compass heading changes and stops after 370°.

### 7. Approaches were cut off after about 58 cm
**v1:** The approach loop has two limits: `maxApproachSteps = 400` (line 277) and `approachSteps > 300` (line 370). The 300 check always triggers first, so 400 is dead code. 300 steps at `MAX_SPEED / 8` is about 58 cm of travel. The jars start 50-162 cm from the search point and the bumper touches at about 25 cm, so for 4 of the 5 starting jar positions the first approach would stop with "Approach taking too long, probably hitting wall!". The robot then backs up and scans again.
**v2:** Checks the thing that message was guessing at. It aborts if the robot moved less than 2 cm in 60 steps (pushing against something), with a hard cap of 900 steps (~176 cm).

### 8. Ending the simulation was ignored
**v1:** Most loops ignore the `-1` that `robot.step()` returns when Webots stops the simulation (for example `delay()` at line 46 and the fixed-count loops at lines 212, 399-426, 444, 456 and 646). The main loop could then keep cycling.
**v2:** All stepping goes through `RobotPlatform.step()`, which throws `SimulationEnded`. Every loop unwinds and the controller exits cleanly.

### 9. Dark grey and brown pixels counted as jar green
**v1:** A pixel is "green" if `R < 120 && G > 35 && B < 120` (line 543). That includes grey (50, 50, 50) and brown (100, 80, 60), so shadows and dark surfaces add to the count.
**v2:** Keeps those thresholds and also requires green to beat red and blue by 10. `VisionTest` checks that v2 accepts only pixels v1 also accepted, and still accepts shaded and washed-out jar colours.

### 10. Smaller fixes
- **Bumper ignored while pausing:** during the 5-step "reassess" pause (lines 357-367) v1 did not read the touch sensor. v2 does.
- **Grab "verification":** v1 printed "Verifying grab..." (line 422) but only waited 20 steps. v2 doesn't claim to verify anything; it logs how many green pixels the camera sees after lifting, which gives data to build a real check from.
- **Unrepeatable runs:** `Math.random()` (line 437) made every run different. v2 uses a seeded `Random`, and `MissionTest` checks that two runs produce identical logs.
- **Compass precision:** v1 cast headings to `int` (line 522), losing up to 1°. v2 keeps doubles. `AnglesTest` checks that the formula is otherwise identical.

---

## Repository fixes

| Problem | Before | After |
|---|---|---|
| Stale compiled controller | `ProjectController3.class` was committed. It was compiled from an older version of the source: it contains `findJar()`, `toStart()` and `hardcodedPath()`, which don't exist in the `.java`. Depending on file timestamps, Webots could run that old code instead of the source. | Deleted (still in git history) and `*.class` ignored. Webots compiles controllers from source. |
| Not a Webots project | Flat files. Webots looks for `controllers/<name>/` next to `worlds/`, and the world loads `../protos/*.proto`. | Standard `worlds/` + `controllers/` + `protos/` layout. |
| Missing robot protos | The world needs `Pioneer3dx.proto` and `Pioneer3Gripper.proto`, which were never committed, so nobody else could open it. | Documented in [`protos/README.md`](../protos/README.md), with an (untested) script that rebuilds them from Webots R2021b. They are still not committed. |
| No tests or CI | none | 46 JUnit tests and GitHub Actions on JDK 11 and 21. CI also checks that the generated benchmark and map are up to date. |

---

## Code structure

| | v1 | v2 |
|---|---|---|
| Layout | 1 file, 765 lines (574 of code), everything `static` | 13 focused classes, 1,293 lines (923 of code; the rest is comments and blank lines) |
| Webots API use | Spread through every method | Only in [`WebotsPlatform`](../controllers/JarCollectorV2/WebotsPlatform.java). Everything else uses the [`RobotPlatform`](../controllers/JarCollectorV2/RobotPlatform.java) interface, so it can be tested without Webots |
| Tunable numbers | Literals scattered through methods | All in [`Config`](../controllers/JarCollectorV2/Config.java), each marked "v1" (unchanged) or "new" |
| Waypoints | Rebuilt inline in 5 methods | Defined once, named, in [`Routes`](../controllers/JarCollectorV2/Routes.java) |
| Duplicate code | Camera-to-array copy twice (lines 217-228, 300-311); position read 7 times | One `captureImage()`, one `positionCm()` |
| Dead code | `WORLD_WIDTH`/`WORLD_HEIGHT`, blue-pixel counting, `GreenDetection.greenLeft/Center/Right`, `moveHardcodedAhead`'s always-`false` return, the 400-step limit | Removed |
| Telemetry | Progress messages without times | Every log line has the simulated time, and each run ends with a mission report: time per phase, per-jar scans and approaches, stalls, timeouts |
| Tests | 0 | 46 |

---

## The original README vs the code

The original README is kept unchanged in [`v1-original-README.md`](v1-original-README.md). A few of its claims didn't match the code, so the new README describes what the code actually does:

| Original README | The v1 code |
|---|---|
| "Operate without GPS, pre-mapped routes, or external guidance systems" | The controller is a Webots Supervisor that reads the robot's exact position from the simulator (`getSelf().getField("translation")`) and follows hand-tuned waypoint routes. Only jar detection relies on the onboard camera and touch sensor |
| Distance sensors "prevent collisions throughout mission" | They're enabled and printed during a start-up debug test (lines 110-119), then never read again |
| Accelerometer for "stability monitoring" | It's in the world file but never enabled or read |
| Blue detection "used for navigation guidance during return phase" | Blue pixels are counted (line 544) but the result is never used |
| "Max speed: 5 m/s", "reduced speed (1 m/s) during precision turns" | `MAX_SPEED = 5` is wheel speed in rad/s, about 0.49 m/s. Turns run at full speed |
| Approach "maximum 400 steps" | 300 (see bug 7) |
| "Rotate 360°" scan | About 271° (see bug 6) |
| Turn when green leads by "15+ pixels" | `EPSILON = 10` |
| "Centimetre-scale integer math eliminates floating-point errors… ~15% more stable" | Positions and distances are `double`; only the waypoint `Point`s are integers |
| "Test Coverage: 100% of core mission paths" | There were no tests |
| Open `simulation/ProjectWorld2025.wbt` | That folder didn't exist |

---

## Switching v2 back to v1 behaviour

Each change is a constant in [`Config.java`](../controllers/JarCollectorV2/Config.java), which makes it easy to A/B test a single change in Webots:

| To get v1's… | Set |
|---|---|
| open-loop driving | `HEADING_HOLD_GAIN = 0` |
| full-speed turns | `TURN_MIN_SPEED = MAX_SPEED` and `TURN_SLOWDOWN_DEG = 0.001` |
| 300-step approach limit | `APPROACH_TIMEOUT_STEPS = 300` |
| exact green test | `GREEN_DOMINANCE_MARGIN = Vision.V1_COMPATIBLE_MARGIN` |

The fixes for bugs 1-3, 8 and 10 have no switch, because they only change what happens when something has already gone wrong.
