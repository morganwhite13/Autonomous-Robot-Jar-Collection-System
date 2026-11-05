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
- 137 JUnit tests, covering 99% of the lines in v2's logic (measured with `tools/run-tests.sh --coverage`). Most run v2's real navigation, search and mission classes on a kinematic model of the Pioneer 3-DX (wheel radius 9.75 cm, axle track 33 cm, 32 ms steps, synthetic 64x64 camera). A small scripted fake covers the edge cases the model can't stage, such as exact pixel counts or the bumper pressing mid-pause.
- Mutation testing (`tools/mutation-test.sh`, [PIT](https://pitest.org/)): of 400 small deliberate bugs injected into v2's logic, the tests catch 376 (94%). The first run caught only 56%, even with 96% line coverage, which is what prompted most of the tests above. The 24 that survive were each checked by hand. 7 can't change behaviour, such as `d < 0` vs `d <= 0` before a wrap-around that maps 360 back to 0. 9 only change log text. 8 only differ when a measured distance or angle lands exactly on a threshold, like 2.000… cm of progress. CI fails below 90%.
- Refactors are checked against a recorded baseline: full mission logs from 6 scenarios (ideal model, planned routes, motor lag and wheel slip, one jar, no jars, blocked start) plus every way an approach can end. The logs, timings, positions and wheel commands must match the previous version exactly.
- The v1 methods being compared, and v1's whole mission loop, were ported line for line into [`tests/LegacyV1.java`](../tests/LegacyV1.java) and [`tests/LegacyV1Mission.java`](../tests/LegacyV1Mission.java) and measured on the same model ([`docs/benchmark-results.md`](benchmark-results.md)). [`LegacyBehaviourTest`](../tests/LegacyBehaviourTest.java) pins each v1 behaviour described here, so the claims fail loudly if they stop being true.
- Route geometry was checked against the obstacles parsed from the `.wbt` file.

**Not done yet:** neither version has been run in Webots as part of this change. Webots could not be launched in the environment this was written in. The robot protos are now included, reconstructed from Webots R2021b and checked statically against R2025a, but they haven't been loaded in Webots either (see [`protos/README.md`](../protos/README.md)). The kinematic model has no inertia, wheel slip or box physics. The next step is running both worlds in Webots, which `tools/run-in-webots.sh` automates. It runs both headless and judges them with the same observer, then writes `docs/webots-results.md` and a map of both driven paths. Every behaviour change can be switched back to v1's behaviour in [`Config.java`](../controllers/JarCollectorV2/Config.java) (see the last section).

---

## Measured on the kinematic model

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="images/before-after-dark.svg">
  <img alt="v1 vs v2 on the same robot model: scan coverage, drift on a 530 cm leg, and distance driven after a failed search" src="images/before-after.svg">
</picture>

| Check | v1 (before) | v2 (after) |
|---|---|---|
| Turn-in-place finishes (6,240 start/target pairs) | 91.7% | **100%** |
| Turn-in-place mean steps when it finishes | 16.7 | 22.9 (slower near the target, on purpose) |
| Rotation actually covered by the "360°" jar scan | **271°** | 371° |
| Scan finds a jar 1 m away, 45° right of the starting heading | no (blind spot) | yes |
| 530 cm leg started 2.9° off (inside the ±3° turn tolerance): distance from the waypoint at the end | **26.8 cm** | 1.1 cm |
| Drive blocked by an immovable obstacle | never returns | stall detected after 3.8 s |
| Longest approach before the time limit aborts it | 59 cm | 176 cm, and aborts early only if not moving |
| Route after a failed search for jars 2-5 | 41 m, first leg through **4 static wooden boxes** | 0.8 m |
| Mission when no jar can be found | retries forever | stops after 3 failed searches with a report |
| A jar slips out of the gripper on the first grab | reports success, holding nothing | with the grab check on (off by default): holding the jar after 2 grabs |
| 30 cm box in the middle of a straight 3 m leg | hits it at 49 cm/s and keeps pushing | hits it at 49 cm/s and gives up after 3.8 s; with the sonar watch on (off by default) never touches it, stopping 24 cm short |
| Whole mission (walls and wooden boxes solid) | **0/5 jars, never finishes**: stuck from 20 s turning on the 5th leg of the box-clearing route | 5/5 jars, 815 s simulated, 0 stalls |

**With motor lag and wheel slip.** The model is ideal, so the benchmark repeats the checks with every wheel command applied 64 ms late and turns reaching only 70% of their ideal rate. Neither version was tuned for these values.

| Check | v1 (before) | v2 (after) |
|---|---|---|
| Turn-in-place finishes | 100% | 100% |
| Rotation actually covered by the "360°" jar scan | **188°** | 371° |
| 530 cm leg started 2.9° off: distance from the waypoint at the end | 26.8 cm | 0.6 cm |
| Whole mission (walls and wooden boxes solid) | **0/5 jars, never finishes**: from 47 s, pushes against a wooden box on the box-clearing route | 5/5 jars, 864 s simulated, 0 stalls |
| Whole mission on 20 robot variants (motor lag 0-3 steps x turn rate 60-100%) | **finishes 0 of 20**; at most 2 jars delivered | finishes 20 of 20, in 807-902 s |

**The whole-mission rows** run a line-for-line port of v1's `main` loop and everything it calls ([`tests/LegacyV1Mission.java`](../tests/LegacyV1Mission.java)) against the real v2 classes, from the world's start pose. Walls and wooden boxes are solid, and the cardboard boxes are left out because the model can't push them. v1 fails in two different ways:
- **On the ideal model it never gets past the box-clearing route** ([bug 4](#4-turns-could-bounce-around-the-target-heading)). v1 turns at full speed, about 5.4° per step, but stops only when its whole-degree compass reading is strictly within 3 of the target. That window is about 5° wide, narrower than one step, so a turn can step back and forth over it forever. On the 5th leg, which heads for 0°, the window is only 4°, because the check doesn't wrap around 0/360, and that's where it happens.
- **With lag and slip its turns finish, but each one overshoots,** because the motors keep turning for the lag after v1 stops them. v1 drives every leg without steering ([bug 5](#5-straight-legs-drifted-off-line)), so the error adds up from leg to leg. At 47 s, on the 8th leg of the box-clearing route, it drives into a wooden box. It has no stall detection ([bug 3](#3-drives-had-no-timeout-or-stall-detection)), so it pushes until the run is stopped.
- **Where v1 gets stuck depends on the details, but it always does.** Across 20 variants of the robot (motor lag 0-3 steps, turn rate 60-100% of ideal) v1 never finishes, and delivers at most 2 jars. v2 finishes all 20. An independent check also varied the robot's collision radius (15 and 19 cm) and got the same picture. The model uses 19 cm, half the Pioneer's 38 cm width.

Two things to keep in mind. The model's camera only draws jars, while in Webots v1's loose green test also counts dark grey and brown pixels such as wooden boxes ([bug 9](#9-dark-grey-and-brown-pixels-counted-as-jar-green)), so the model probably flatters v1's search. And on the model a blocked robot stays put, so "pushes forever" is literal there. In Webots, contact could push it sideways far enough to finish the leg.

![Animated replay of v1 and v2 running the whole mission with motor lag and wheel slip: v1 drifts off its route while clearing boxes and pushes against a wooden box from 47 s; v2 delivers all five in 864 s](images/mission-replay.svg)

`LegacyBehaviourTest` pins both results. This is a model with no inertia, and the [demo videos](../README.md) show the robot collecting jars in Webots. Whether that was v1 or the older v0 isn't known yet; v0 turned at 1 rad/s and wouldn't overshoot like this. A Webots run of both versions (`tools/run-in-webots.sh`) is the real test.

The full generated tables are in [`benchmark-results.md`](benchmark-results.md). Regenerate them with `tools/run-tests.sh`.

---

## Bugs fixed

### 1. A failed search sent the robot straight across the arena (serious)
**v1:** `main` picks the route to the search point with `if (jarsToGo == 5)` (line 130). Only jar 1 uses the short route. For jars 2-5 it uses `navigateFromDropoffToSearch()`, which assumes the robot is at the drop-off. After a failed search (lines 144-148) the robot is still at the search point, so the route's first leg (line 588) is a straight 17.8 m line from the jar bay to the drop-off. That line passes through four wooden boxes, which have no mass in the world file and so cannot be pushed. Combined with bug 3 below, the robot could push against one forever.

```mermaid
flowchart TD
    A[Jar search fails] --> B{jarsToGo == 5?}
    B -- "yes (jar 1)" --> C[Short route back to the search point]
    B -- "no (jars 2-5)" --> D[Drop-off-to-search route]
    D --> E["First leg: straight line from the jar bay to the drop-off, 17.8 m"]
    E --> F[Runs into static wooden boxes]
    F --> G[No stall detection: can push against one forever]
    style D fill:#fde2d6,stroke:#eb6834
    style E fill:#fde2d6,stroke:#eb6834
    style F fill:#fde2d6,stroke:#eb6834
    style G fill:#fde2d6,stroke:#eb6834
```

**v2:** [`Mission`](../controllers/JarCollectorV2/Mission.java) tracks whether the robot is at the drop-off and takes the 0.8 m hop back to the search point after a failed search. It also drops v1's extra random move after a failed search (line 147): the robot drives straight back to the same search point anyway, so it only risked knocking a jar. Random moves inside a search (when a full scan sees nothing) are kept. The red dotted line on the [mission map](images/mission-map.svg) shows the old route.

<details>
<summary>Code: v1 vs v2</summary>

v1 (`ProjectController3.java`, lines 129-148):
```java
// Go to search area
if (jarsToGo == 5) {
  navigateToSearchArea();
} else {
  navigateFromDropoffToSearch();   // assumes the robot is at the drop-off
}
boolean success = intelligentJarSearch(timeStep);
if (success) {
  ...
} else {
  System.out.println("CRITICAL: Failed to find jar after all attempts!");
  moveRandomly(timeStep);          // then loops back to the "if" above
}
```

v2 (`Mission.java`):
```java
List<Vec2> route = atDropoff ? Routes.DROPOFF_TO_SEARCH : Routes.TO_SEARCH_POINT;
nav.followRoute(route);
atDropoff = false;

if (!jars.searchAndGrab()) {
  stats.recordFailedSearch();
  if (++failedInARow >= Config.MAX_CONSECUTIVE_FAILED_SEARCHES) {
    break;                          // stop and print the mission report
  }
  continue;                         // still at the search point: short route next time
}
```
</details>

### 2. Endless retries
**v1:** `while (jarsToGo > 0)` (line 126) has no attempt limit, so a jar that cannot be found makes the program run forever.
**v2:** Stops after `MAX_CONSECUTIVE_FAILED_SEARCHES` (3) failures in a row and prints what it achieved.

### 3. Drives had no timeout or stall detection
**v1:** `moveHardcodedAhead` (lines 709-715) loops until the robot is far enough from where it started. If the robot is blocked, that never happens.
**v2:** [`Navigator.driveTo`](../controllers/JarCollectorV2/Navigator.java) checks progress every 40 steps. If the robot moved less than 2 cm, it reports a stall and continues with the next waypoint. Each drive also has a hard timeout. Both are counted in the mission report.

### 4. Turns could bounce around the target heading
**v1:** `makeHardcodedTurn` (lines 722-765) always turns at full speed, about 5.4° per 32 ms step, and stops only when the integer heading lands strictly inside `finalAngle ± 3` (line 748). That window is 5 values wide, so a step can jump over it, reverse, and jump back. On the model, 8.3% of start/target pairs never finished. An independent check with a different heading-only model ([PR #1](https://github.com/morganwhite13/Autonomous-Robot-Jar-Collection-System/pull/1)) found about the same rate: 135 of 1,296 turns never settled. The check also doesn't wrap around 0/360: a target of 0° only accepts 0-2°, and 359° only accepts 357-359°. It also always turns for one step before checking, even when already aligned.

How often this bites depends on how fast the robot really turns. When slip slows the turn to 70% of the ideal rate, each step is small enough to land in the window and all of v1's turns finish. So in Webots it may show up as occasional extra swinging rather than a hang. The wrap-around gap remains either way.

**v2:** Uses a seam-safe heading error, full speed down to 30° from the target, then proportional slow-down. It checks before moving and has a timeout. It finished 100% of pairs, at about 0.2 s extra per turn.

<details>
<summary>Code: v1 vs v2</summary>

v1 (`ProjectController3.java`, lines 746-760), always at full speed:
```java
while (robot.step(timeStep) != -1) {
  int degrees = (getCompassReadingInDegrees(compass) + 360) % 360;
  if (degrees > finalAngle - 3 && degrees < finalAngle + 3)   // not wrap-safe
    break;
  int distTurningRight = (degrees - finalAngle + 360) % 360;
  int distTurningLeft = (finalAngle - degrees + 360) % 360;
  if (distTurningRight < distTurningLeft) {
    leftMotor.setVelocity(MAX_SPEED);  rightMotor.setVelocity(-MAX_SPEED);
  } else {
    leftMotor.setVelocity(-MAX_SPEED); rightMotor.setVelocity(MAX_SPEED);
  }
}
```

v2 (`Navigator.java`), check first, slow down near the target, give up eventually:
```java
for (int i = 0; i < Config.TURN_TIMEOUT_STEPS; i++) {
  double error = Angles.delta(robot.headingDeg(), targetDeg);      // in (-180, 180]
  if (Math.abs(error) <= Config.HEADING_TOLERANCE_DEG) {
    robot.stop();
    return true;
  }
  double speed = clamp(maxSpeed * Math.abs(error) / Config.TURN_SLOWDOWN_DEG, minSpeed, maxSpeed);
  if (error > 0) robot.drive(-speed, speed); else robot.drive(speed, -speed);
  robot.step();
}
```
</details>

### 5. Straight legs drifted off line
**v1:** Drives open-loop. A turn that ends 2.9° off (allowed by the ±3° tolerance) puts the robot 26.8 cm from the waypoint at the end of the 530 cm leg. Box pushing adds more error.
**v2:** Steers towards the waypoint while driving: a proportional correction capped at 20% of wheel speed, switched off in the last 20 cm. It ends 1.1 cm away.

### 6. The "360°" scan covered about 270°
**v1:** `rotateAndFindGreen` rotates for a fixed 200 steps at `MAX_SPEED / 4` (lines 206-212). With the Pioneer's wheel geometry and the default 32 ms step that is about 271°. Since the robot always turns left, a jar slightly right of the starting heading is never in view.
**v2:** Adds up the compass heading changes and stops after 370°. This also makes it independent of how fast the robot actually turns: with 30% wheel slip, v1's fixed 200 steps cover only 188°, while v2 still covers a full turn.

### 7. Approaches were cut off after about 58 cm
**v1:** The approach loop has two limits: `maxApproachSteps = 400` (line 277) and `approachSteps > 300` (line 370). The 300 check always triggers first, so 400 is dead code. 300 steps at `MAX_SPEED / 8` is about 58 cm of travel. The jars start 50-162 cm from the search point and the bumper touches at about 25 cm, so for 4 of the 5 starting jar positions the first approach would stop with "Approach taking too long, probably hitting wall!". The robot then backs up and scans again.
**v2:** Checks the thing that message was guessing at. It aborts if the robot moved less than 2 cm in 60 steps (pushing against something), with a hard cap of 900 steps (~176 cm).

### 8. Ending the simulation was ignored
**v1:** Most loops ignore the `-1` that `robot.step()` returns when Webots stops the simulation (for example `delay()` at line 46 and the fixed-count loops at lines 212, 399-426, 444, 456 and 646). The main loop could then keep cycling.
**v2:** All stepping goes through `RobotPlatform.step()`, which throws `SimulationEnded`. Every loop unwinds and the controller exits cleanly.

### 9. Dark grey and brown pixels counted as jar green
**v1:** A pixel is "green" if `R < 120 && G > 35 && B < 120` (line 543). That includes grey (50, 50, 50) and brown (100, 80, 60), so shadows and dark surfaces add to the count.
**v2:** Keeps those thresholds and also requires green to beat red and blue by 10. `VisionTest` checks that v2 accepts only pixels v1 also accepted, and still accepts shaded and washed-out jar colours. v1's pixel-count thresholds (20, 15, 30) were tuned on v1's counts, so during approaches v2 logs both counts ("v1 rule: N px"). The first Webots run will show whether the stricter test ever loses a jar that v1 would still see.

### 10. Smaller fixes
- **Bumper ignored while pausing:** during the 5-step "reassess" pause (lines 357-367) v1 did not read the touch sensor. v2 does.
- **Grab "verification":** v1 printed "Verifying grab..." (line 422) but only waited 20 steps, so a jar that slipped out of the fingers was carried "home" anyway and counted as delivered. v2 has a real check, **off by default** until a Webots run confirms its threshold (`Config.VERIFY_GRABS`):
  - **How it works:** after lifting, it reads the finger motors' force feedback. Fingers that closed on a jar stop against it and push at their limit, while fingers that closed on nothing stop pushing.
  - **What it does:** if the fingers are using less than half their maximum force, v2 lets go, backs up and tries again.
  - **Logging:** every grab logs this "grip effort", whether or not the check is on, so the first Webots run provides the data to set the threshold.
  - **On the model:** with a jar that slips out on the first grab, v1 reports success while holding nothing, and v2 with the check on is holding the jar after its second grab. Over a whole mission with one slipped jar, v2 with the check off believes it delivered 5 jars while only 4 reach the pads; with the check on, all 5 do (`MissionTest`).
- **Unrepeatable runs:** `Math.random()` (line 437) made every run different. v2 uses a seeded `Random`, and `MissionTest` checks that two runs produce identical logs.
- **Compass precision:** v1 cast headings to `int` (line 522), losing up to 1°. v2 keeps doubles. `AnglesTest` checks that the formula is otherwise identical.
- **Sonars never used:** v1 enabled six sonars and only printed them at start-up (lines 110-119). v2 has an optional **sonar watch**, off by default until it's tuned in Webots (`Config.SONAR_WATCH`):
  - **What it does:** on travel legs, if the two front sonars (so3, so4) hear an echo closer than the waypoint, v2 slows down in proportion and stops 25 cm short instead of driving into it.
  - **Where it's off:** the box-clearing route, which has to push boxes, and placement, which drives up to the pads.
  - **The model's sonar:** it uses the proto's geometry and Webots' rule that a sonar gets no echo from a surface hit more than 22.5° off its normal.
  - **What changes:** on a clear course the watch changes nothing, and the runs are step-for-step identical (`MissionTest`). With a box in the way it never touches the box. After stopping it skips to the next waypoint, as it does after a stall; it doesn't plan a way around yet.

---

## Repository fixes

| Problem | Before | After |
|---|---|---|
| Stale compiled controller | `ProjectController3.class` was committed. It was compiled from an older version of the source: it contains `findJar()`, `toStart()` and `hardcodedPath()`, which don't exist in the `.java`. Webots never compiles Java controllers by itself; it runs the `.class` it finds. So anyone who opened the repository ran that old code, not the source. | Deleted (still in git history) and `*.class` ignored. Build from source with `tools/build-controllers.sh` or Webots' Build button (the README now says so). |
| Not a Webots project | Flat files. Webots looks for `controllers/<name>/` next to `worlds/`, and the world loads `../protos/*.proto`. | Standard `worlds/` + `controllers/` + `protos/` layout. |
| Missing robot protos | The world needs `Pioneer3dx.proto` and `Pioneer3Gripper.proto`, which were never committed, so nobody else could open it. | Included in `protos/`, reconstructed from Webots R2021b. Every node type and field is checked statically against R2025a; not yet loaded in Webots. See [`protos/README.md`](../protos/README.md). |
| No tests or CI | none | 137 JUnit tests (99% line coverage of v2's logic, 94% mutation score) and GitHub Actions on JDK 11 and 21. CI also checks that the generated benchmark and figures are up to date. |

---

## Code structure

| | v1 | v2 |
|---|---|---|
| Layout | 1 file, 765 lines (574 of code), everything `static` | 14 focused files (13 classes and the `RobotPlatform` interface), 1,657 lines (1,164 of code; the rest is comments and blank lines) |
| Webots API use | Spread through every method | Only in [`WebotsPlatform`](../controllers/JarCollectorV2/WebotsPlatform.java). Everything else uses the [`RobotPlatform`](../controllers/JarCollectorV2/RobotPlatform.java) interface, so it can be tested without Webots |
| Tunable numbers | Literals scattered through methods | All in [`Config`](../controllers/JarCollectorV2/Config.java), each marked "v1" (unchanged) or "new" |
| Waypoints | Rebuilt inline in 5 methods | Defined once, named, in [`Routes`](../controllers/JarCollectorV2/Routes.java) |
| Duplicate code | Camera-to-array copy twice (lines 217-228, 300-311); position read 7 times | One `captureImage()`, one `positionCm()` |
| Dead code | `WORLD_WIDTH`/`WORLD_HEIGHT`, blue-pixel counting, `GreenDetection.greenLeft/Center/Right`, `moveHardcodedAhead`'s always-`false` return, the 400-step limit | Removed |
| Telemetry | Progress messages without times | Every log line has the simulated time, and each run ends with a mission report: time per phase, per-jar scans and approaches, stalls, timeouts |
| Tests | 0 | 137, covering 99% of v2's logic lines (87% including the two classes that only run inside Webots) |
| Mutation testing ([PIT](https://pitest.org/)) | n/a | 94% of injected bugs caught (376/400); at least 90% enforced in CI (`tools/mutation-test.sh`) |
| Static analysis ([SpotBugs](https://spotbugs.github.io/), maximum effort, every priority) | 10 findings, all low severity: 5 unread fields, 3 `Math.floor` calls on constants, an int made via `nextDouble`, a field-naming slip | 0, enforced in CI (`tools/static-analysis.sh`) |

---

## The original README vs the code

The original README is kept unchanged in [`v1-original-README.md`](v1-original-README.md). Checking it against the code turned up something unexpected: **the repository held two different controllers.**

- **v1** is the committed source, `ProjectController3.java`, with `intelligentJarSearch`, `carefullyApproachGreen` and `executeGrab`.
- **v0** is an earlier version that survived only as the stale `ProjectController3.class` (deleted here; still in git history at `d598324`). Decompiled, it has `findJar()`, `toStart()`, `hardcodedPath()` and `placeJar()`, green thresholds of R < 80, G > 50 and B < 80, and `EPSILON = 15`. Its `toStart()` slows to `MAX_SPEED = 1` for turns.

The original README mixes descriptions of both, so several claims that look wrong for v1 were accurate for v0. The demo videos could show either version; only the author knows which. The new README describes the code that's actually in the repository.

| Original README | v0 (stale `.class`) | v1 (committed source) |
|---|---|---|
| "Operate without GPS, pre-mapped routes, or external guidance systems" | No: both are Webots Supervisors that read the robot's exact position from the simulator and follow hand-tuned waypoint routes. Only jar detection uses the onboard camera and touch sensor. | Same as v0 |
| Distance sensors "prevent collisions throughout mission" | No: enabled, never read | No: printed in a start-up debug test (lines 110-119), never used |
| Accelerometer for "stability monitoring" | No: never enabled or read | No: never enabled or read |
| Blue detection "used for navigation guidance during return phase" | No: blue counts are computed in `findJar()` but never used | No: counted (line 544), never used |
| "Max speed: 5 m/s", "reduced speed (1 m/s) during precision turns" | Yes, apart from the units: 5 and 1 are wheel speeds in rad/s (about 0.49 and 0.1 m/s) | 5 rad/s; turns always run at full speed |
| "Three-tier fallback" detection; turn when green leads by "15+ pixels" | Yes: `EPSILON = 15`, then a relative comparison, then a raw count above 10 | Partly: one margin, `EPSILON = 10` |
| Approach "maximum 400 steps"; "rotate 360°" scan | Not applicable: v0 approaches differently | 300 steps effective (bug 7); the scan turns about 271° (bug 6) |
| "Centimetre-scale integer math eliminates floating-point errors… ~15% more stable" | No: `double` maths | No: positions and distances are `double`; only the waypoint `Point`s are integers |
| "Test Coverage: 100% of core mission paths" | No tests | No tests |
| Open `simulation/ProjectWorld2025.wbt` | That folder didn't exist | That folder didn't exist |

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

Three v2 additions are **off by default** because they haven't been tried in Webots yet: the A*-planned routes (`USE_PLANNED_ROUTES`), the grab check (`VERIFY_GRABS`) and the sonar watch (`SONAR_WATCH`).
