import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Random;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class JarHandlerTest {

  @BeforeAll
  static void silenceLog() {
    Log.useSink(line -> { });
  }

  private static JarHandler handler(KinematicRobot robot) {
    MissionStats stats = new MissionStats(robot::elapsedMs);
    return new JarHandler(robot, new Navigator(robot, stats), stats, new Random(1));
  }

  private static Vec2 polar(double distance, double bearingDeg) {
    double rad = Math.toRadians(bearingDeg);
    return Vec2.of(distance * Math.cos(rad), distance * Math.sin(rad));
  }

  @Test
  void scanTurnsAtLeastAFullCircle() {
    KinematicRobot robot = new KinematicRobot(Vec2.of(0, 0), 270);
    handler(robot).scan();
    assertTrue(Math.abs(robot.cumulativeTurnDeg) >= 360, "turned " + robot.cumulativeTurnDeg);
  }

  @Test
  void scanStillTurnsAFullCircleWhenTheWheelsSlip() {
    KinematicRobot robot = new KinematicRobot(Vec2.of(0, 0), 270).withImperfections(2, 0.5);
    handler(robot).scan();
    assertTrue(Math.abs(robot.cumulativeTurnDeg) >= 360, "turned " + robot.cumulativeTurnDeg);
  }

  @Test
  void scanFindsAJarThatV1sScanCannotSee() {
    KinematicRobot robot = new KinematicRobot(Vec2.of(0, 0), 0);
    robot.jars.add(polar(100, -45)); // just right of the starting heading
    JarHandler.Detection best = handler(robot).scan();
    assertNotNull(best);
    assertTrue(Angles.within(best.headingDeg, 315, 15), "best at " + best.headingDeg);
  }

  @Test
  void scanPrefersTheBiggerCloserJar() {
    KinematicRobot robot = new KinematicRobot(Vec2.of(0, 0), 0);
    robot.jars.add(polar(60, 90));
    robot.jars.add(polar(200, 200));
    JarHandler.Detection best = handler(robot).scan();
    assertTrue(Angles.within(best.headingDeg, 90, 15), "best at " + best.headingDeg);
  }

  @Test
  void approachReachesAJarBeyondV1sTimeLimitAndGrabsIt() {
    KinematicRobot robot = new KinematicRobot(Vec2.of(0, 0), 0);
    robot.jars.add(Vec2.of(150, 0));
    assertTrue(handler(robot).approachAndGrab());
    assertNotNull(robot.heldJar);
    assertTrue(robot.lift <= Config.CARRY_LIFT && robot.fingers <= Config.FINGERS_CLOSED);
  }

  @Test
  void approachSteersOntoAnOffCentreJar() {
    KinematicRobot robot = new KinematicRobot(Vec2.of(0, 0), 0);
    robot.jars.add(polar(80, 15));
    assertTrue(handler(robot).approachAndGrab());
  }

  @Test
  void approachGivesUpWhenPushingAgainstAWall() {
    KinematicRobot robot = new KinematicRobot(Vec2.of(0, 0), 0);
    robot.jars.add(Vec2.of(200, 0));
    robot.blocked = p -> p.x > 50;
    assertFalse(handler(robot).approachAndGrab());
    assertNull(robot.heldJar);
    assertTrue(robot.stepsTaken() < Config.APPROACH_TIMEOUT_STEPS);
  }

  @Test
  void approachGivesUpWhenNothingIsVisible() {
    KinematicRobot robot = new KinematicRobot(Vec2.of(0, 0), 0);
    assertFalse(handler(robot).approachAndGrab());
    // Same as v1: abort on the 31st frame without the jar; the 30 frames before it each also
    // paused 5 steps to reassess.
    int pausedFrames = Config.APPROACH_MAX_STEPS_WITHOUT_JAR;
    assertEquals(pausedFrames * (1 + Config.REASSESS_PAUSE_STEPS) + 1, robot.stepsTaken());
  }

  @Test
  void grabNeedsContact() {
    KinematicRobot robot = new KinematicRobot(Vec2.of(0, 0), 0);
    assertFalse(handler(robot).grab());
  }

  @Test
  void placeDropsTheJarAtItsPad() {
    KinematicRobot robot = new KinematicRobot(Routes.DROPOFF_ENTRY, 90);
    robot.heldJar = Vec2.of(0, 0);
    robot.fingers = Config.FINGERS_CLOSED;
    handler(robot).place(2);
    assertNull(robot.heldJar);
    Vec2 dropped = robot.deliveredJars.get(0);
    assertTrue(dropped.distanceTo(Vec2.of(-967, Routes.slotY(2))) < 15, "dropped at " + dropped);
  }

  @Test
  void grabUsesV1sTimingAndLeavesTheJarHeld() {
    KinematicRobot robot = new KinematicRobot(Vec2.of(0, 0), 0);
    robot.jars.add(Vec2.of(20, 0));
    assertTrue(handler(robot).grab());
    int v1Steps = Config.GRAB_SETTLE_STEPS + Config.GRAB_CLOSE_STEPS + Config.GRAB_LIFT_STEPS
        + Config.GRAB_HOLD_STEPS;
    assertEquals(250, v1Steps);
    assertEquals(v1Steps, robot.stepsTaken());
    assertNotNull(robot.heldJar);
    assertEquals(Config.FINGERS_CLOSED, robot.fingers);
    assertEquals(Config.CARRY_LIFT, robot.lift); // lower than v1's LIFT_UP: see Config
    assertTrue(robot.isStopped());
  }

  @Test
  void placeOpensLowersAndBacksOut() {
    KinematicRobot robot = new KinematicRobot(Routes.DROPOFF_ENTRY, 90);
    robot.heldJar = Vec2.of(0, 0);
    robot.fingers = Config.FINGERS_CLOSED;
    robot.lift = Config.LIFT_UP;
    handler(robot).place(0);
    assertEquals(Config.FINGERS_OPEN, robot.fingers);
    assertEquals(Config.LIFT_DOWN, robot.lift);
    assertTrue(robot.isStopped());
    double backedUp = Config.BACKUP_AFTER_RELEASE_STEPS * 0.032 * Config.MANOEUVRE_SPEED
        * KinematicRobot.WHEEL_RADIUS_CM;
    assertEquals(-936 + backedUp, robot.positionCm().x, 4);
    assertEquals(Routes.slotY(0), robot.positionCm().y, 4);
  }

  @Test
  void approachThatNeverReachesTheJarTimesOutStopped() {
    ScriptedRobot robot = new ScriptedRobot();
    robot.center = 40; // always in view, dead ahead
    robot.orbitRadiusCm = 100;
    robot.orbitDegPerStep = 0.5; // always moving, so never "not making progress"
    assertFalse(handler(robot).approachAndGrab());
    assertEquals(Config.APPROACH_TIMEOUT_STEPS, robot.steps);
    assertTrue(robot.isStopped());
  }

  @Test
  void aFarJarIsApproachedFasterThanANearOne() {
    double[] far = firstApproachCommand(0); // jar's base high in the image: far away
    double[] near = firstApproachCommand(61); // base near the bottom: about 60 cm
    assertArrayEquals(new double[] {Config.FAST_APPROACH_SPEED, Config.FAST_APPROACH_SPEED}, far, 1e-12);
    assertArrayEquals(new double[] {Config.APPROACH_SPEED, Config.APPROACH_SPEED}, near, 1e-12);
  }

  private double[] firstApproachCommand(int firstRow) {
    ScriptedRobot robot = new ScriptedRobot();
    robot.center = 40;
    robot.firstRow = firstRow;
    robot.touchedAtStep = step -> step >= 2;
    handler(robot).approachAndGrab();
    return robot.commands.get(0);
  }

  @Test
  void bumperPressedDuringTheReassessPauseStillGrabs() {
    ScriptedRobot robot = new ScriptedRobot();
    robot.left = 5;
    robot.center = 5;
    robot.right = 5; // 15 px: visible, but no third leads, so the robot pauses to reassess
    robot.touchedAtStep = step -> step >= 3;
    MissionStats stats = new MissionStats(robot::elapsedMs);
    JarHandler handler = new JarHandler(robot, new Navigator(robot, stats), stats, new Random(1));
    assertTrue(handler.approachAndGrab());
    assertEquals(3 + 250, robot.steps);
    assertEquals(Config.CARRY_LIFT, robot.lift);
  }

  @Test
  void failedApproachBacksUpBeforeTheNextScan() {
    KinematicRobot robot = new KinematicRobot(Vec2.of(0, 0), 0);
    robot.jars.add(Vec2.of(200, 0));
    robot.blocked = p -> p.x > 60; // a wall between the robot and the jar
    MissionStats stats = new MissionStats(robot::elapsedMs);
    stats.beginJar(1);
    JarHandler handler = new JarHandler(robot, new Navigator(robot, stats), stats, new Random(1));
    assertFalse(handler.searchAndGrab());
    assertEquals(Config.MAX_SCANS_PER_SEARCH, stats.jars().get(0).scans);
    assertEquals(Config.MAX_SCANS_PER_SEARCH, stats.jars().get(0).approaches);
    assertTrue(robot.positionCm().x < 60 - 20, "backed away from the wall: " + robot.positionCm());
  }

  @Test
  void scanThatSeesNothingWandersSomewhereElse() {
    KinematicRobot robot = new KinematicRobot(Vec2.of(0, 0), 0);
    robot.trace = new java.util.ArrayList<>();
    MissionStats stats = new MissionStats(robot::elapsedMs);
    stats.beginJar(1);
    assertFalse(new JarHandler(robot, new Navigator(robot, stats), stats, new Random(1)).searchAndGrab());
    assertEquals(Config.MAX_SCANS_PER_SEARCH, stats.jars().get(0).scans);
    assertEquals(0, stats.jars().get(0).approaches);
    double driven = 0;
    for (int i = 1; i < robot.trace.size(); i++) {
      driven += robot.trace.get(i - 1).distanceTo(robot.trace.get(i));
    }
    double oneWander = Config.WANDER_STEPS * 0.032 * Config.MANOEUVRE_SPEED * KinematicRobot.WHEEL_RADIUS_CM;
    assertEquals(Config.MAX_SCANS_PER_SEARCH * oneWander, driven, 1);
  }

  @Test
  void scanNeedsMoreThanTwentyGreenPixelsAndSamplesEveryFifthStep() {
    ScriptedRobot faint = new ScriptedRobot();
    faint.center = 20;
    assertNull(handler(faint).scan());
    int expectedFrames = (faint.steps + Config.SCAN_SAMPLE_EVERY_STEPS - 1) / Config.SCAN_SAMPLE_EVERY_STEPS;
    assertEquals(expectedFrames, faint.captures);
    assertTrue(faint.isStopped());

    ScriptedRobot visible = new ScriptedRobot();
    visible.center = 21;
    JarHandler.Detection best = handler(visible).scan();
    assertNotNull(best);
    assertEquals(21 + Config.SCAN_CENTER_BONUS, best.score);
    // Equal scores never replace the first detection, as in v1.
    assertEquals(visible.turnDegPerStep, best.headingDeg, 1e-9);
  }

  private static JarHandler handler(ScriptedRobot robot) {
    MissionStats stats = new MissionStats(robot::elapsedMs);
    return new JarHandler(robot, new Navigator(robot, stats), stats, new Random(1));
  }

  @Test
  void approachSteersWithV1sWheelFactors() {
    double s = Config.APPROACH_SPEED;
    ScriptedRobot toTheLeft = new ScriptedRobot();
    toTheLeft.firstRow = 61; // near: v1's creeping speed
    toTheLeft.left = 40;
    toTheLeft.touchedAtStep = step -> step >= 2;
    handler(toTheLeft).approachAndGrab();
    assertArrayEquals(new double[] {s * Config.APPROACH_INNER_FACTOR, s * Config.APPROACH_OUTER_FACTOR},
        toTheLeft.commands.get(0), 1e-12);
    ScriptedRobot toTheRight = new ScriptedRobot();
    toTheRight.firstRow = 61;
    toTheRight.right = 40;
    toTheRight.touchedAtStep = step -> step >= 2;
    handler(toTheRight).approachAndGrab();
    assertArrayEquals(new double[] {s * Config.APPROACH_OUTER_FACTOR, s * Config.APPROACH_INNER_FACTOR},
        toTheRight.commands.get(0), 1e-12);
  }

  @Test
  void exactlyFifteenPixelsStillCountsAsSeeingTheJar() {
    ScriptedRobot robot = new ScriptedRobot();
    robot.left = 5;
    robot.center = 5;
    robot.right = 5; // never "lost", so the approach only ends when the robot isn't moving
    assertFalse(handler(robot).approachAndGrab());
    int frames = Config.APPROACH_STALL_WINDOW_STEPS;
    assertEquals(frames * (1 + Config.REASSESS_PAUSE_STEPS), robot.steps);
    // ...and the robot stood still during every reassess pause
    for (int step = 2; step <= 6; step++) {
      assertFalse(robot.movingDuring.get(step - 1), "moving during pause step " + step);
    }
    assertTrue(robot.isStopped());
  }

  @Test
  void abortedApproachesLeaveTheWheelsStopped() {
    KinematicRobot nothing = new KinematicRobot(Vec2.of(0, 0), 0);
    handler(nothing).approachAndGrab();
    assertTrue(nothing.isStopped());
    KinematicRobot walled = new KinematicRobot(Vec2.of(0, 0), 0);
    walled.jars.add(Vec2.of(200, 0));
    walled.blocked = p -> p.x > 50;
    handler(walled).approachAndGrab();
    assertTrue(walled.isStopped());
  }

  @Test
  void grabStopsAMovingRobotFirst() {
    KinematicRobot robot = new KinematicRobot(Vec2.of(0, 0), 0);
    robot.jars.add(Vec2.of(20, 0));
    robot.drive(Config.APPROACH_SPEED, Config.APPROACH_SPEED);
    assertTrue(handler(robot).grab());
    assertEquals(0, robot.positionCm().x, 1e-9);
  }

  @Test
  void scanThatCannotTurnGivesUpAtItsTimeout() {
    KinematicRobot robot = new KinematicRobot(Vec2.of(0, 0), 0).withImperfections(0, 0.0);
    assertNull(handler(robot).scan());
    assertEquals(Config.SCAN_TIMEOUT_STEPS, robot.stepsTaken());
    assertTrue(robot.isStopped());
  }

  @Test
  void wanderTurnsByTheSeededRandomAngle() {
    KinematicRobot robot = new KinematicRobot(Vec2.of(0, 0), 0);
    int expected = new Random(1).nextInt(2 * Config.WANDER_MAX_TURN_DEG) - Config.WANDER_MAX_TURN_DEG;
    handler(robot).wander();
    assertTrue(Angles.within(robot.headingDeg(), expected, Config.HEADING_TOLERANCE_DEG),
        "heading " + robot.headingDeg() + ", expected " + expected);
  }

  @Test
  void losingTheJarWhileSteeringStopsTheWheels() {
    ScriptedRobot robot = new ScriptedRobot();
    robot.left = 11; // fewer than 15 px (lost), but still enough to steer left
    assertFalse(handler(robot).approachAndGrab());
    assertEquals(Config.APPROACH_MAX_STEPS_WITHOUT_JAR + 1, robot.steps);
    assertTrue(robot.isStopped());
  }

  @Test
  void anUnclearViewStopsAMovingRobotBeforeThePause() {
    ScriptedRobot robot = new ScriptedRobot();
    robot.sectorsAtStep = step -> step == 1 ? new int[] {40, 0, 0} : new int[] {5, 5, 5};
    robot.touchedAtStep = step -> step >= 4;
    assertTrue(handler(robot).approachAndGrab());
    assertTrue(robot.movingDuring.get(1), "steering left during step 2");
    assertFalse(robot.movingDuring.get(2), "still moving during the reassess pause");
  }

  @Test
  void placeWaitsForTheJarToSettleBeforeBackingOut() {
    KinematicRobot robot = new KinematicRobot(Routes.DROPOFF_ENTRY, 90);
    robot.heldJar = Vec2.of(0, 0);
    robot.trace = new java.util.ArrayList<>();
    handler(robot).place(0);
    List<Vec2> trace = robot.trace;
    int backingOut = trace.size() - Config.BACKUP_AFTER_RELEASE_STEPS;
    int stillSteps = 0;
    for (int i = backingOut - 1; i > 0 && trace.get(i).equals(trace.get(i - 1)); i--) {
      stillSteps++;
    }
    assertEquals(Config.RELEASE_STEPS, stillSteps);
  }

  // Grab check (Config.VERIFY_GRABS, off by default).

  private static JarHandler checkingHandler(RobotPlatform robot, MissionStats stats) {
    return new JarHandler(robot, new Navigator(robot, stats), stats, new Random(1), true);
  }

  @Test
  void grabCheckLetsGoAndTriesAgainWhenTheJarSlipsOut() {
    KinematicRobot robot = new KinematicRobot(Vec2.of(0, 0), 0);
    robot.jars.add(Vec2.of(60, 0));
    robot.grabsThatMiss = 1;
    MissionStats stats = new MissionStats(robot::elapsedMs);
    assertTrue(checkingHandler(robot, stats).searchAndGrab());
    assertNotNull(robot.heldJar);
    assertEquals(1, stats.missedGrabs());
  }

  @Test
  void aMissedGrabOpensAndLowersTheGripper() {
    KinematicRobot robot = new KinematicRobot(Vec2.of(0, 0), 0);
    robot.jars.add(Vec2.of(20, 0));
    robot.grabsThatMiss = 1;
    MissionStats stats = new MissionStats(robot::elapsedMs);
    assertFalse(checkingHandler(robot, stats).grab());
    assertEquals(Config.FINGERS_OPEN, robot.fingers);
    assertEquals(Config.LIFT_DOWN, robot.lift);
    int v1Steps = Config.GRAB_SETTLE_STEPS + Config.GRAB_CLOSE_STEPS + Config.GRAB_LIFT_STEPS
        + Config.GRAB_HOLD_STEPS;
    assertEquals(v1Steps + Config.RELEASE_STEPS, robot.stepsTaken());
  }

  @Test
  void withoutTheGrabCheckASlippedJarStillCountsAsGrabbed() {
    KinematicRobot robot = new KinematicRobot(Vec2.of(0, 0), 0);
    robot.jars.add(Vec2.of(60, 0));
    robot.grabsThatMiss = 1;
    assertTrue(handler(robot).searchAndGrab()); // the default, like v1
    assertNull(robot.heldJar);
  }

  @Test
  void grabCheckTrustsAGripItCannotMeasure() {
    ScriptedRobot robot = new ScriptedRobot(); // gripEffort() is NaN
    robot.touchedAtStep = step -> true;
    MissionStats stats = new MissionStats(robot::elapsedMs);
    assertTrue(checkingHandler(robot, stats).grab());
    assertEquals(0, stats.missedGrabs());
  }

  @Test
  void grabCheckThresholdIsInclusive() {
    ScriptedRobot exactly = new ScriptedRobot() {
      @Override
      public double gripEffort() {
        return Config.GRIP_EFFORT_HOLDING;
      }
    };
    exactly.touchedAtStep = step -> true;
    assertTrue(checkingHandler(exactly, new MissionStats(exactly::elapsedMs)).grab());
  }

  @Test
  void sightingsOfTheSameJarAreMergedAndOneOffsDropped() {
    java.util.List<Vec2> jars = JarHandler.cluster(java.util.List.of(
        Vec2.of(100, 0), Vec2.of(110, 5), Vec2.of(95, -4), // one jar, three sightings
        Vec2.of(300, 300), Vec2.of(305, 310), // another, two
        Vec2.of(-200, 50))); // seen once: ignored
    assertEquals(2, jars.size());
    assertTrue(jars.get(0).distanceTo(Vec2.of(101.7, 0.3)) < 0.1, jars.get(0).toString());
    assertTrue(jars.get(1).distanceTo(Vec2.of(302.5, 305)) < 0.1, jars.get(1).toString());
  }

  @Test
  void aScanMapsTheJarsItSeesAndAGrabForgetsThatOne() {
    KinematicRobot robot = new KinematicRobot(Vec2.of(0, 0), 0);
    robot.jars.add(Vec2.of(120, 0));
    robot.jars.add(Vec2.of(0, 140));
    JarHandler handler = handler(robot);
    handler.scan();
    java.util.List<Vec2> seen = handler.rememberedJars();
    assertEquals(2, seen.size(), seen.toString());
    for (Vec2 jar : robot.jars) {
      assertTrue(seen.stream().anyMatch(s -> s.distanceTo(jar) < 15), jar + " not in " + seen);
    }
    assertTrue(handler.searchAndGrab());
    assertEquals(1, handler.rememberedJars().size());
    // The nearer jar, (120, 0), was fetched first; the one left is (0, 140).
    assertTrue(handler.rememberedJars().get(0).distanceTo(Vec2.of(0, 140)) < 15,
        handler.rememberedJars().toString());
  }

  /** A scripted robot whose finger force follows a list of readings, then stays at the last. */
  private static ScriptedRobot withGripEfforts(double... readings) {
    return new ScriptedRobot() {
      private int next;

      @Override
      public double gripEffort() {
        return readings[Math.min(next++, readings.length - 1)];
      }
    };
  }

  @Test
  void gripWatchNeedsALongRunOfLowReadingsAndRemembersTheLowest() {
    int confirm = Config.DROP_CONFIRM_STEPS;
    double[] readings = new double[1 + (confirm - 1) + 1 + 1 + confirm];
    int i = 0;
    readings[i++] = 1.0;
    for (int k = 0; k < confirm - 1; k++) {
      readings[i++] = 0.2; // a dip one reading too short to count
    }
    readings[i++] = Config.GRIP_EFFORT_HOLDING; // the grip recovers: exactly at the threshold
    readings[i++] = Double.NaN; // can't tell: counts as holding
    for (int k = 0; k < confirm; k++) {
      readings[i++] = 0.1; // gone for good
    }
    ScriptedRobot robot = withGripEfforts(readings);
    JarHandler.GripWatch watch = handler(robot).gripWatch();
    assertTrue(Double.isNaN(watch.lowest()));
    for (int k = 0; k < readings.length - 1; k++) {
      assertTrue(watch.getAsBoolean(), "reading " + k);
    }
    assertFalse(watch.getAsBoolean(), "the last of " + confirm + " low readings in a row");
    assertEquals(0.1, watch.lowest(), 1e-12);
  }

  @Test
  void aScanThatLeadsToAGrabForgetsTheGrabbedJar() {
    KinematicRobot robot = new KinematicRobot(Vec2.of(0, 0), 0);
    robot.jars.add(Vec2.of(120, 0));
    robot.jars.add(Vec2.of(0, 140));
    JarHandler handler = handler(robot);
    assertTrue(handler.searchAndGrab()); // nothing remembered yet: scans, then grabs one
    assertEquals(1, handler.rememberedJars().size());
    Vec2 left = handler.rememberedJars().get(0);
    assertTrue(left.distanceTo(robot.jars.get(0)) < 15, left + " vs " + robot.jars);
  }

  @Test
  void aPlacementAbandonedOnTheWayLetsGoWithoutBackingOut() {
    KinematicRobot robot = new KinematicRobot(Vec2.of(-850, 607), 180);
    robot.setFingers(Config.FINGERS_CLOSED);
    assertFalse(handler(robot).place(0, () -> false));
    assertEquals(Config.FINGERS_OPEN, robot.fingers);
    assertEquals(Config.LIFT_DOWN, robot.lift);
    assertTrue(robot.isStopped());
    assertTrue(robot.positionCm().distanceTo(Vec2.of(-850, 607)) < 1e-9, "moved: " + robot.positionCm());
  }

  @Test
  void aFarJarIsApproachedFasterButSteeredAsSharplyAsV1Did() {
    double s = Config.APPROACH_SPEED;
    double boost = Config.FAST_APPROACH_SPEED - s;
    ScriptedRobot left = new ScriptedRobot();
    left.left = 40; // far away: green high in the frame
    left.touchedAtStep = step -> step >= 2;
    handler(left).approachAndGrab();
    assertArrayEquals(new double[] {s * Config.APPROACH_INNER_FACTOR + boost,
        s * Config.APPROACH_OUTER_FACTOR + boost}, left.commands.get(0), 1e-12);
    ScriptedRobot right = new ScriptedRobot();
    right.right = 40;
    right.touchedAtStep = step -> step >= 2;
    handler(right).approachAndGrab();
    assertArrayEquals(new double[] {s * Config.APPROACH_OUTER_FACTOR + boost,
        s * Config.APPROACH_INNER_FACTOR + boost}, right.commands.get(0), 1e-12);
  }
}
