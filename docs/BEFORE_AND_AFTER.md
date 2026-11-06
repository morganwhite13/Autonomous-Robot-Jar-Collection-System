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

**In Webots (real physics):**
- **The whole mission:** both versions ran it in Webots R2025a, headless, through `tools/run-in-webots.sh`. An observer supervisor judges both by the same rule: a jar counts once it rests in the drop-off lane. Results are below and in [`webots-results.md`](webots-results.md).
- **Six shuffled worlds:** one world is thin evidence, so `tools/webots-sweep.sh` also runs both versions on six shuffled worlds. Each places the jars at random in the jar bay and nudges the robot's start pose by up to 8 cm and 6° (`tools/WorldShuffle.java`). v1 delivered 0 of 30 jars. v2 delivered all 30, finishing every world in 721-765 s ([`webots-sweep.md`](webots-sweep.md)). Four of v2's fixes ([bugs 12-14](#12-carrying-the-jar-5-cm-up-let-it-slip-out-found-in-webots) and the detour on a stall) came from these runs.
- **On every push:** CI runs v2's whole mission in Webots and fails unless all five jars are delivered.
- **Repeatability:** rerunning a world reproduces its log and path, give or take a single 32 ms step. One rerun drifted by one step after 268 s.
- **Fixes outside the controllers:** getting both versions running took two fixes to the reconstructed robot protos (below).

**On a kinematic model (fast, and run in CI):**
- Both controllers compile against the real Webots R2025a `Controller.jar`. CI compiles them against API stubs whose signatures were copied from that jar.
- 181 JUnit tests, covering 99% of the lines in v2's logic (measured with `tools/run-tests.sh --coverage`). Most run v2's real navigation, search and mission classes on a kinematic model of the Pioneer 3-DX (wheel radius 9.75 cm, axle track 33 cm, 32 ms steps, synthetic 64x64 camera). A small scripted fake covers the edge cases the model can't stage, such as exact pixel counts or the bumper pressing mid-pause.
- Mutation testing (`tools/mutation-test.sh`, [PIT](https://pitest.org/)) injects small deliberate bugs into v2's logic and checks the tests catch them. Of 817 such bugs, they catch 742 (91%). The first run caught only 56%, even with 96% line coverage, which is what prompted most of the tests above. Most survivors can't change behaviour, only change log text, or only differ when a measured distance or angle lands exactly on a threshold, like 2.000… cm of progress. CI fails below 90%.
- Refactors are checked against a recorded baseline: full mission logs from 6 scenarios (ideal model, planned routes, motor lag and wheel slip, one jar, no jars, blocked start) plus every way an approach can end. The logs, timings, positions and wheel commands must match the previous version exactly.
- The v1 methods being compared, and v1's whole mission loop, were ported line for line into [`tests/LegacyV1.java`](../tests/LegacyV1.java) and [`tests/LegacyV1Mission.java`](../tests/LegacyV1Mission.java) and measured on the same model ([`docs/benchmark-results.md`](benchmark-results.md)). [`LegacyBehaviourTest`](../tests/LegacyBehaviourTest.java) pins each v1 behaviour described here, so the claims fail loudly if they stop being true.
- Route geometry was checked against the obstacles parsed from the `.wbt` file.

**Still not checked:**
- **The robot is a reconstruction.** The protos are Cyberbotics' Webots R2021b Pioneer 3-DX, because the originals were never committed. The original robot may have differed. For example, the [demo videos](../README.md) show jars being collected, and on this reconstruction v1 never delivers one.
- **Six worlds is a small sample.** A run in a world v2 hasn't seen could still find a new failure, the way the shuffled worlds found bugs 12-14.
- **Tipped-over jars.** A jar lying on its side can be grabbed like a standing one if it lies across the robot's path. One lying end-on to the robot crashed Webots' physics as soon as the robot turned while gripping it, so v2 can't be tested on that. v2 carries jars low, so the ones that slip out land upright.
- **Switching back:** every behaviour change can be switched back to v1's behaviour in [`Config.java`](../controllers/JarCollectorV2/Config.java) (see the last section).

---

## Measured in Webots

### The original world

Both versions run the whole mission in Webots R2025a, with real physics, the same world (apart from the controller name), the same reconstructed robot, and the same observer judging deliveries. Generated by `tools/run-in-webots.sh --limit 1500 --frames 3 v1 v2 v2-hand-tuned` into [`webots-results.md`](webots-results.md). `v2-hand-tuned` is v2 on v1's carry and return waypoints instead of its A*-planned routes.

| | v1 | v2 | v2-hand-tuned |
|---|---|---|---|
| Jars delivered | 0/5 | 5/5 | 5/5 |
| Jars the controller says it delivered | 4 | 5 | 5 |
| Mission time (last jar delivered) | did not finish (stopped at 1500 s) | 738 s (12.3 min) | 827 s (13.8 min) |
| Jar delivery times (s) | - | 177, 311, 446, 584, 738 | 189, 342, 499, 658, 827 |
| Stuck in one place from | 1204 s to the end | - | - |
| Distance driven | 281 m | 247 m | 276 m |
| Approach aborted | 7 | 0 | 0 |
| Whole search failed | 1 | 0 | 0 |
| Drive stalled or timed out (v2 detects these; v1 cannot) | 0 | 0 | 0 |

![Paths driven in Webots: v1 in orange ends pinned against the wall at the far end of the jar bay; v2 in blue and v2 on the hand-tuned routes in green both make five round trips to the drop-off](images/webots-paths.svg)

**What v1 did.** Every failure below was predicted from the code:
- **It aimed at the wrong things.** Its green test also counts the floor and grey walls ([bug 9](#9-dark-grey-and-brown-pixels-counted-as-jar-green)). Every scan saw 2,600-3,700 "green" pixels at 36-53°, pointing away from the jars, and it turned there.
- **It grabbed nothing, four times, and counted each one.** Driving that way, the bumper eventually touched a wall or box. v1 closed the gripper on nothing, carried nothing to the drop-off, "placed" it and printed "Jars remaining: 4", then 3, 2 and 1. It never checks its grip ([bug 13](#13-a-jar-that-fell-out-on-the-way-was-still-counted-found-in-webots)).
- **Then it got stuck for good.** On jar 5 all three approaches failed. v1 then took the route meant to start from the drop-off, but from the search point ([bug 1](#1-a-failed-search-sent-the-robot-straight-across-the-arena-serious)). It ended up pressed against the wall at the far end of the jar bay. With no stall detection ([bug 3](#3-drives-had-no-timeout-or-stall-detection)), it was still pushing there, creeping about 1 cm every 10 s, when the run was stopped.
- **Its runs differ.** v1 picks its random turns with unseeded `Math.random()`. An earlier run counted 3 empty deliveries and got stuck against a wooden box from 745 s; another got stuck elsewhere at 659 s. The pattern is the same every time.

**What v2 did.** It cleared the boxes in 90 s, scanned twice in the whole mission (the [jar map](#16-speed-ups) found jars 2-4 without scanning), grabbed every jar on the first approach and delivered all five in 738 s. It reported no stalls, timeouts, failed searches, missed grabs or drops. On v1's hand-tuned routes it took 827 s.

![Animated replay at 30x speed of v1 and v2 running the whole mission in Webots: v1 makes four empty trips to the drop-off and ends pinned against a wall; v2 delivers all five jars in 738 s](images/webots-replay.svg)

<details>
<summary>v2 seen from above (GIF, 25 s at 30x speed)</summary>

![Top-down view of v2 in Webots: it pushes the boxes aside, then fetches the five jars from the jar bay one at a time and lines them up at the drop-off](images/v2-webots.gif)

</details>

### Six shuffled worlds

One world can be lucky. `tools/webots-sweep.sh` reruns both versions on six shuffled worlds: the jars go to random spots in the jar bay, at least 25 cm apart and clear of obstacles, and the robot's start pose is nudged by up to 8 cm and 6°. Same observer, 1500 s limit. Full tables are in [`webots-sweep.md`](webots-sweep.md).

| | v1 | v2 |
|---|---|---|
| Delivered all 5 jars | 0 of 6 worlds | 6 of 6 worlds |
| Jars delivered | 0 of 30 | 30 of 30 |
| Jars the controller says it delivered | 22 | 30 |
| Mission time when all 5 were delivered (median, range) | - | 744 s (721-765 s) |
| Runs stuck in one place at the end | 5 | 0 |

v1 fails the same way in every world. It aims at the floor and walls, counts empty trips to the drop-off as deliveries, and in 5 worlds of 6 ends stuck in one place.

**How v2 got there.** The shuffled worlds found problems the original world never showed. Each row adds to the one above:

| v2 version | All 5 delivered | Jars delivered (claimed) | Median mission time |
|---|---|---|---|
| First Webots-ready v2: slower carry turns ([bug 11](#11-full-speed-turns-threw-the-jar-out-of-the-gripper-found-in-webots)) | 1 of 6 worlds | 22 of 30 (claimed 30) | - |
| + carry the jar low, notice a drop, drive around stalls ([bugs 12](#12-carrying-the-jar-5-cm-up-let-it-slip-out-found-in-webots), [13](#13-a-jar-that-fell-out-on-the-way-was-still-counted-found-in-webots), [15](#15-something-in-the-way-stopped-a-leg-for-good)) | 5 of 6 | 29 of 30 | 889 s |
| + jar map, faster approach ([speed-ups](#16-speed-ups)) | 6 of 6 | 30 of 30 | 837 s |
| + A*-planned routes | 5 of 6 | 29 of 30 | 740 s |
| + steer towards a jar no third of the image clearly holds ([bug 14](#14-a-small-far-jar-between-two-thirds-of-the-image-froze-the-approach-found-in-webots)), go around a blocked approach; sonar watch and grab check on: **v2** | **6 of 6** | **30 of 30** | **744 s** |

The planned routes' miss was bug 14, in the one world where the last jar sat 1.7 m from the search point. On the hand-tuned routes, that world had needed four tries to get the same jar. The sonar watch and grab check were each first tried alone on the jar-map version, on the same six worlds. The sonar watch never stopped for nothing, and its results were identical to the runs without it. The grab check never rejected a real grab.

**With something in the way.** Two more runs used one of the shuffled worlds with an extra wooden block between the search point and the jars:
- **A 30 cm tall block:** v2 still delivered all five, in 981 s.
  - It stopped short of the block twice on the sonars, then drove around it.
  - Once an approach was blocked, and v2 went around the block to the jar.
  - Three times the bumper pressed against the block instead of a jar, so the gripper closed on its edge. The grab check caught one of these. The grip watch caught the other two within a second of setting off.
- **A 6 cm block,** which the sonars and the camera can't see: v2 went around it while approaching the first jar and grabbed it. Then the carry route ran over the block, the jar was knocked out, and the robot got caught on the block. Low obstacles are listed under [future improvements](../README.md#-future-improvements).

### What it took

v2 didn't work in Webots on the first try. Getting it there found these problems, in this order:

| Problem | Symptom | Fix |
|---|---|---|
| The protos had their header changed from R2021b to R2025a | Webots skipped its conversion of pre-2022 files. The wheels' collision cylinders lay across the wheels, so the robot dropped onto its caster every half turn and stalled for about 2.5 s with the wheels spinning. The camera looked out of the right side, rolled 90°, so jars were seen while they were beside the robot. Both versions failed for reasons that had nothing to do with their code. | Original header restored; a test pins it ([`protos/README.md`](../protos/README.md)) |
| Full-speed turns while carrying a jar | The jar flew out within 0.6 s of the first turn towards the drop-off and landed on its side next to the search point, where it couldn't be grabbed again. v1 has the same problem: it turns at full speed too. | v2 turns at a third of full speed while carrying ([bug 11](#11-full-speed-turns-threw-the-jar-out-of-the-gripper-found-in-webots)) |
| Wheels kept turning after v2 finished | Webots applies motor commands on the next step, so a mission that ended right after backing up left the robot reversing. | v2 stops and steps once before exiting |
| The shuffled worlds | Jars slipped out on the way and were still counted; a far jar froze an approach. | [Bugs 12-14](#12-carrying-the-jar-5-cm-up-let-it-slip-out-found-in-webots) |

**Keep in mind:** the robot is a reconstruction. The original protos were never committed; these are Cyberbotics' R2021b Pioneer 3-DX. The demo videos show jars being collected, so the original setup may have suited v1 better, or the videos may show the older v0.

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
| A jar slips out of the gripper on the first grab | reports success, holding nothing | holding the jar after 2 grabs (grab check) |
| 30 cm box in the middle of a straight 3 m leg | hits it at 49 cm/s and keeps pushing | never touches it: the sonar watch stops 24 cm short (without the watch: hits it at 49 cm/s, stall detected after 3.8 s) |
| Whole mission (walls and wooden boxes solid) | **0/5 jars, never finishes**: stuck from 20 s turning on the 5th leg of the box-clearing route | 5/5 jars, 732 s simulated, 0 stalls |

**With motor lag and wheel slip.** The model is ideal, so the benchmark repeats the checks with every wheel command applied 64 ms late and turns reaching only 70% of their ideal rate. Neither version was tuned for these values.

| Check | v1 (before) | v2 (after) |
|---|---|---|
| Turn-in-place finishes | 100% | 100% |
| Rotation actually covered by the "360°" jar scan | **188°** | 371° |
| 530 cm leg started 2.9° off: distance from the waypoint at the end | 26.8 cm | 0.6 cm |
| Whole mission (walls and wooden boxes solid) | **0/5 jars, never finishes**: from 47 s, pushes against a wooden box on the box-clearing route | 5/5 jars, 788 s simulated, 0 stalls |
| Whole mission on 20 robot variants (motor lag 0-3 steps x turn rate 60-100%) | **finishes 0 of 20**; at most 2 jars delivered | finishes 20 of 20, in 728-822 s |

**The whole-mission rows** run a line-for-line port of v1's `main` loop and everything it calls ([`tests/LegacyV1Mission.java`](../tests/LegacyV1Mission.java)) against the real v2 classes, from the world's start pose. Walls and wooden boxes are solid, and the cardboard boxes are left out because the model can't push them. v1 fails in two different ways:
- **On the ideal model it never gets past the box-clearing route** ([bug 4](#4-turns-could-bounce-around-the-target-heading)). v1 turns at full speed, about 5.4° per step, but stops only when its whole-degree compass reading is strictly within 3 of the target. That window is about 5° wide, narrower than one step, so a turn can step back and forth over it forever. On the 5th leg, which heads for 0°, the window is only 4°, because the check doesn't wrap around 0/360, and that's where it happens.
- **With lag and slip its turns finish, but each one overshoots,** because the motors keep turning for the lag after v1 stops them. v1 drives every leg without steering ([bug 5](#5-straight-legs-drifted-off-line)), so the error adds up from leg to leg. At 47 s, on the 8th leg of the box-clearing route, it drives into a wooden box. It has no stall detection ([bug 3](#3-drives-had-no-timeout-or-stall-detection)), so it pushes until the run is stopped.
- **Where v1 gets stuck depends on the details, but it always does.** Across 20 variants of the robot (motor lag 0-3 steps, turn rate 60-100% of ideal) v1 never finishes, and delivers at most 2 jars. v2 finishes all 20. An independent check also varied the robot's collision radius (15 and 19 cm) and got the same picture. The model uses 19 cm, half the Pioneer's 38 cm width.

The model is kinder to v1 than Webots was, in two ways. Its camera only draws jars, while in Webots v1's loose green test also counted the floor and walls and aimed it away from every jar ([bug 9](#9-dark-grey-and-brown-pixels-counted-as-jar-green)). And on the model a blocked robot stays put. In Webots, contact pushed v1 far enough to finish the box-clearing route, and it got stuck later instead (see [Measured in Webots](#measured-in-webots)).

![Animated replay of v1 and v2 running the whole mission with motor lag and wheel slip: v1 drifts off its route while clearing boxes and pushes against a wooden box from 47 s; v2 delivers all five in 788 s](images/mission-replay.svg)

`LegacyBehaviourTest` pins both results. v2's model times are for its default A*-planned routes; on v1's hand-tuned routes the ideal mission takes 822 s. The model can't throw a jar unless told to, but Webots showed that full-speed turns do ([bug 11](#11-full-speed-turns-threw-the-jar-out-of-the-gripper-found-in-webots)).

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
**v2:** Keeps those thresholds and also requires green to beat red and blue by 10. `VisionTest` checks that v2 accepts only pixels v1 also accepted, and still accepts shaded and washed-out jar colours. v1's pixel-count thresholds (20, 15, 30) were tuned on v1's counts, so during approaches v2 logs both counts ("v1 rule: N px"). In Webots the stricter test found every jar with one scan and counted nothing elsewhere. v1's test counted up to 3,300 "green" pixels where there was no jar, mostly floor and walls, and its scans pointed it away from the jars.

### 10. Smaller fixes
- **Bumper ignored while pausing:** during the 5-step "reassess" pause (lines 357-367) v1 did not read the touch sensor. v2 does.
- **Grab "verification":** v1 printed "Verifying grab..." (line 422) but only waited 20 steps, so a jar that slipped out of the fingers was carried "home" anyway and counted as delivered. v2 has a real check (`Config.VERIFY_GRABS`):
  - **How it works:** after lifting, it reads the finger motors' force feedback. Fingers that closed on a jar stop against it and push at their limit, while fingers that closed on nothing stop pushing.
  - **What it does:** if the fingers are using less than half their maximum force, v2 lets go, backs up and tries again.
  - **In Webots:** real grabs logged a grip effort of 1.00, and when a jar was thrown out of the gripper the finger force dropped to 0 at once, so the 0.5 threshold sits well between the two. On six shuffled worlds the check never rejected a real grab, and the runs matched the ones without it.
  - **On the model:** with a jar that slips out on the first grab, v1 reports success while holding nothing, and v2 with the check on is holding the jar after its second grab. Over a whole mission with one slipped jar, v2 with the check off believes it delivered 5 jars while only 4 reach the pads; with the check on, all 5 do (`MissionTest`).
- **Unrepeatable runs:** `Math.random()` (line 437) made every run different. v2 uses a seeded `Random`, and `MissionTest` checks that two runs produce identical logs.
- **Compass precision:** v1 cast headings to `int` (line 522), losing up to 1°. v2 keeps doubles. `AnglesTest` checks that the formula is otherwise identical.
- **Sonars never used:** v1 enabled six sonars and only printed them at start-up (lines 110-119). v2 has a **sonar watch** (`Config.SONAR_WATCH`):
  - **What it does:** on travel legs, if the two front sonars (so3, so4) hear an echo closer than the waypoint, v2 slows down in proportion and stops 25 cm short instead of driving into it.
  - **Where it's off:** the box-clearing route, which has to push boxes, and placement, which drives up to the pads.
  - **The model's sonar:** it uses the proto's geometry and Webots' rule that a sonar gets no echo from a surface hit more than 22.5° off its normal.
  - **What changes:** on a clear course the watch changes nothing. On the model the runs are step-for-step identical (`MissionTest`), and on six shuffled Webots worlds it never stopped for nothing. With a box in the way it never touches the box: it stops short and drives around it ([bug 15](#15-something-in-the-way-stopped-a-leg-for-good)).

### 11. Full-speed turns threw the jar out of the gripper (found in Webots)
**v1:** After a grab, the first move towards the drop-off is a turn in place at full speed (`makeHardcodedTurn`, line 722), and so is every turn along the way.
**Found in Webots:** a supervisor logged the jar's position while the robot grabbed it and turned. The grab itself is solid: the fingers squeeze at their 10 N limit and the jar lifts 5 cm, upright. But turning at 5 rad/s throws it out within 0.6 s, and it lands on its side. At 2.5 rad/s it stays in the fingers through the turn, a full-speed straight drive and four hard stops; 1.67 rad/s holds it through the turn and drive too. v2's first Webots runs lost every jar this way: it grabbed them, then drove to the drop-off empty-handed.
**v2:** Routes driven with a jar in the gripper (to the drop-off, and lining up with the pad) turn at most at `Config.CARRY_TURN_SPEED`, a third of full speed. That's the speed v1 itself used for its manoeuvres. It costs about 12 s per delivery. `LegacyBehaviourTest` shows v1's first carry turn throwing the jar on a model told to throw it above 2.5 rad/s, and `MissionTest` shows v2 delivering all five on that model.

The next fixes came from running both versions on six shuffled worlds in Webots ([below](#measured-in-webots)), where the jars sit in different places and the robot starts slightly off its usual pose.

### 12. Carrying the jar 5 cm up let it slip out (found in Webots)
**v1:** Right after the grab it lifts the jar to the top of the lift, 5 cm (line 416), and carries it like that.
**Found in Webots:** Even with slower carry turns (bug 11), v2 lost 8 of 30 jars on the way across the six worlds. The jars slid out of the fingers on hard stops and turns. A supervisor test carried jars through four 4 m runs and 180° turns. Lifted 5 cm, 1 jar of 3 stayed in the gripper. Lifted 0.8 cm, all 6 did, including grabs 4 and 7 cm off centre. A jar that slips from that height lands upright between the fingers.
**v2:** Carries the jar 0.8 cm up (`Config.CARRY_LIFT`). It raises it to v1's 5 cm only for the short run onto the pad.

### 13. A jar that fell out on the way was still counted (found in Webots)
**v1:** After the grab it drives the whole carry route, "places" whatever is in the gripper, and counts a delivery (lines 138-143). It never checks that it's still holding the jar.
**Found in Webots:** In the sweep above, v2 copied this and claimed 30 deliveries when only 22 jars had arrived.
**v2:** Watches the finger force while carrying (`Config.WATCH_GRIP_WHILE_CARRYING`):
- **How it tells:** fingers holding a jar push at their limit. In Webots the force dips below half for up to 9 steps while the jar is still held, and stays down once the jar is gone. So 30 low steps in a row, about 1 s, count as a drop.
- **What it does:** it opens the gripper, plans a route back to the search point and fetches the next jar. The report counts dropped jars.
- **On the model:** a jar thrown out on the first carry turn is noticed, and the other four are delivered (`MissionTest`).

### 14. A small, far jar between two thirds of the image froze the approach (found in Webots)
**v1:** It steers towards a third of the camera image only if that third leads both others by more than 10 green pixels. With no clear lead, it drives straight only if more than 30 pixels show. Otherwise it stops for 5 steps "to reassess" (lines 355-367).
**Found in Webots:** On one shuffled world, every approach to the last jar, 1.7 m from the search point, stopped dead.
- The camera saw the jar as 5 px in the centre third and 15 px in the right one. That is neither a clear lead nor 30 px, so the robot stopped.
- Standing still, it saw the same picture and stopped again, until the progress check gave up 22 s later.
- A camera filming the search point from above showed nothing in the robot's way.
- With v1's rule, v2 delivered only 4 of 5 jars in that world on the planned routes. On the hand-tuned routes it took four tries.

**v2:** When the jar is in sight (at least 15 px) but no third clearly leads, it steers towards the outer third with more green, or straight if they're level (`Config.STEER_WHEN_UNCLEAR`). Below 15 px the jar counts as lost and v1's pause stays. On the robot model, v2 reaches a jar 2 m away on the line between two thirds (`JarHandlerTest`).

### 15. Something in the way stopped a leg for good
**v1:** Pushes against it forever ([bug 3](#3-drives-had-no-timeout-or-stall-detection)).
**v2 before:** Noticed the stall after 3.8 s and skipped to the next waypoint. That only helps if the next leg happens to miss the obstacle.
**v2:** Drives around it (`Config.DETOUR_AROUND_STALLS`):
- **On a stall:** it backs off and assumes a 40 cm square just ahead of where it stopped is blocked. It plans a way around that square and the walls and wooden boxes with A* ([`ArenaMap`](../controllers/JarCollectorV2/ArenaMap.java), [`RoutePlanner`](../controllers/JarCollectorV2/RoutePlanner.java)) and follows it.
- **With the sonar watch:** the same happens when the sonars stop it short.
- **On an approach:** if something stops the robot before it reaches a jar that is on the map ([speed-ups](#16-speed-ups)), v2 goes around it to 80 cm from the jar and approaches again (`Config.APPROACH_AGAIN_FROM_CM`).
- **In Webots:** a wooden box was added on the carry route at (600, 122). v2 stalled against it, drove around it and delivered all five jars. With the sonar watch on, it stopped 24 cm short of the box and then drove around it.

### 16. Speed-ups
- **Every search scanned a full turn.** v1 turns a full circle at the search point for every jar. During a scan, v2 now maps where each jar it sees is (`Config.REMEMBER_JARS`):
  - **Distance:** the lowest green row in the image gives the distance. The camera is 16.5 cm up and 14 cm ahead of the robot's centre, with a focal length of 77.25 px, all measured in Webots.
  - **Bearing:** the green's column gives the bearing.
  - **Map entries:** sightings within 25 cm of each other are one jar, and a jar seen twice goes on the map.
  - **Next search:** it turns straight to the nearest mapped jar and checks that it's there. A mission now needs 2-3 full scans instead of at least 5.
- **Approaches crept the whole way.** v1 approaches at an eighth of full speed, about 6 cm/s, whether the jar is 20 cm or 2 m away. While the jar is more than 75 cm away, v2 adds the same extra speed to both wheels, up to a third of full speed (`Config.FAST_APPROACH_SPEED`). It turns no more sharply than v1's creep, so the jar stays in view.
- **Hand-tuned routes.** v2 now carries and returns on shorter A*-planned routes by default (`Config.USE_PLANNED_ROUTES`, [route planner](route-planner.md)). They keep the same 30 cm clearance from walls and wooden boxes.

---

## Repository fixes

| Problem | Before | After |
|---|---|---|
| Stale compiled controller | `ProjectController3.class` was committed. It was compiled from an older version of the source: it contains `findJar()`, `toStart()` and `hardcodedPath()`, which don't exist in the `.java`. Webots never compiles Java controllers by itself; it runs the `.class` it finds. So anyone who opened the repository ran that old code, not the source. | Deleted (still in git history) and `*.class` ignored. Build from source with `tools/build-controllers.sh` or Webots' Build button (the README now says so). |
| Not a Webots project | Flat files. Webots looks for `controllers/<name>/` next to `worlds/`, and the world loads `../protos/*.proto`. | Standard `worlds/` + `controllers/` + `protos/` layout. |
| Missing robot protos | The world needs `Pioneer3dx.proto` and `Pioneer3Gripper.proto`, which were never committed, so nobody else could open it. | Included in `protos/`, reconstructed from Webots R2021b and run in Webots R2025a. They must keep their R2021b header so Webots converts them; a test checks it. See [`protos/README.md`](../protos/README.md). |
| No tests or CI | none | 181 JUnit tests (99% line coverage of v2's logic, 91% mutation score) and GitHub Actions on JDK 11 and 21. CI also runs v2's whole mission in Webots, and checks that the generated benchmark and figures are up to date. |

---

## Code structure

| | v1 | v2 |
|---|---|---|
| Layout | 1 file, 765 lines (574 of code), everything `static` | 16 focused files (15 classes and the `RobotPlatform` interface), 2,634 lines (1,862 of code; the rest is comments and blank lines) |
| Webots API use | Spread through every method | Only in [`WebotsPlatform`](../controllers/JarCollectorV2/WebotsPlatform.java). Everything else uses the [`RobotPlatform`](../controllers/JarCollectorV2/RobotPlatform.java) interface, so it can be tested without Webots |
| Tunable numbers | Literals scattered through methods | All in [`Config`](../controllers/JarCollectorV2/Config.java), each marked "v1" (unchanged) or "new" |
| Waypoints | Rebuilt inline in 5 methods | Defined once, named, in [`Routes`](../controllers/JarCollectorV2/Routes.java) |
| Duplicate code | Camera-to-array copy twice (lines 217-228, 300-311); position read 7 times | One `captureImage()`, one `positionCm()` |
| Dead code | `WORLD_WIDTH`/`WORLD_HEIGHT`, blue-pixel counting, `GreenDetection.greenLeft/Center/Right`, `moveHardcodedAhead`'s always-`false` return, the 400-step limit | Removed |
| Telemetry | Progress messages without times | Every log line has the simulated time, and each run ends with a mission report: time per phase, per-jar scans and approaches, stalls, timeouts |
| Tests | 0 | 181, covering 99% of v2's logic lines (91% including the two classes that only run inside Webots) |
| Mutation testing ([PIT](https://pitest.org/)) | n/a | 91% of injected bugs caught (742/817); at least 90% enforced in CI (`tools/mutation-test.sh`) |
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

Each change is a constant in [`Config.java`](../controllers/JarCollectorV2/Config.java), which makes it easy to A/B test a single change in Webots (`tools/run-in-webots.sh --set NAME=VALUE`):

| To get v1's… | Set |
|---|---|
| open-loop driving | `HEADING_HOLD_GAIN = 0` |
| full-speed turns | `TURN_MIN_SPEED = MAX_SPEED` and `TURN_SLOWDOWN_DEG = 0.001` |
| 300-step approach limit | `APPROACH_TIMEOUT_STEPS = 300` |
| exact green test | `GREEN_DOMINANCE_MARGIN = Vision.V1_COMPATIBLE_MARGIN` |
| full-speed turns while carrying a jar (throws it out in Webots) | `CARRY_TURN_SPEED = MAX_SPEED` |
| 5 cm carry height (jars slip out in Webots) | `CARRY_LIFT = LIFT_UP` |
| blindness to a jar falling out on the way | `WATCH_GRIP_WHILE_CARRYING = false` |
| stop-and-reassess when no third of the image clearly leads | `STEER_WHEN_UNCLEAR = false` |
| a full scan for every jar | `REMEMBER_JARS = false` |
| the slow approach all the way | `FAST_APPROACH_SPEED = APPROACH_SPEED` |
| hand-tuned carry and return routes | `USE_PLANNED_ROUTES = false` (or `tools/run-in-webots.sh v2-hand-tuned`) |
| no grab check | `VERIFY_GRABS = false` |
| sonars unused | `SONAR_WATCH = false` |
| skipping a stalled leg instead of driving around (v2's earlier behaviour; v1 pushes forever) | `DETOUR_AROUND_STALLS = false` |

The fixes for bugs 1-3, 8 and 10 have no switch, because they only change what happens when something has already gone wrong.
