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
    assertTrue(robot.lift <= Config.LIFT_UP && robot.fingers <= Config.FINGERS_CLOSED);
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
    assertEquals(Config.LIFT_UP, robot.lift);
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
    KinematicRobot robot = new KinematicRobot(Vec2.of(0, 0), 0);
    robot.jars.add(Vec2.of(210, 0)); // visible all the way, but out of reach in 900 steps
    assertFalse(handler(robot).approachAndGrab());
    assertEquals(Config.APPROACH_TIMEOUT_STEPS, robot.stepsTaken());
    assertTrue(robot.isStopped());
    assertNull(robot.heldJar);
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
    assertEquals(Config.LIFT_UP, robot.lift);
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
    toTheLeft.left = 40;
    toTheLeft.touchedAtStep = step -> step >= 2;
    handler(toTheLeft).approachAndGrab();
    assertArrayEquals(new double[] {s * Config.APPROACH_INNER_FACTOR, s * Config.APPROACH_OUTER_FACTOR},
        toTheLeft.commands.get(0), 1e-12);
    ScriptedRobot toTheRight = new ScriptedRobot();
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
}
