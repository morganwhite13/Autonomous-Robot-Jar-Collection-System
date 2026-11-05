import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class NavigatorTest {

  @BeforeAll
  static void silenceLog() {
    Log.useSink(line -> { });
  }

  private static Navigator nav(KinematicRobot robot) {
    return new Navigator(robot, new MissionStats(robot::elapsedMs));
  }

  @Test
  void turnFinishesWithinToleranceForEveryStartAndTarget() {
    for (int start = 0; start < 360; start += 7) {
      for (int target = 0; target < 360; target += 11) {
        KinematicRobot robot = new KinematicRobot(Vec2.of(0, 0), start);
        assertTrue(nav(robot).turnTo(target, Config.MAX_SPEED), start + " -> " + target);
        assertTrue(Angles.within(robot.headingDeg(), target, Config.HEADING_TOLERANCE_DEG));
        assertTrue(robot.stepsTaken() <= 60, start + " -> " + target + " took " + robot.stepsTaken());
      }
    }
  }

  @Test
  void turnStillFinishesWithMotorLagAndWheelSlip() {
    for (int start = 0; start < 360; start += 13) {
      for (int target = 0; target < 360; target += 17) {
        KinematicRobot robot = new KinematicRobot(Vec2.of(0, 0), start).withImperfections(3, 0.6);
        assertTrue(nav(robot).turnTo(target, Config.MAX_SPEED), start + " -> " + target);
        assertTrue(Angles.within(robot.headingDeg(), target, Config.HEADING_TOLERANCE_DEG + 2));
      }
    }
  }

  @Test
  void turnGivesUpInsteadOfHangingWhenTheRobotCannotRotate() {
    KinematicRobot robot = new KinematicRobot(Vec2.of(0, 0), 0).withImperfections(0, 0.0);
    MissionStats stats = new MissionStats(robot::elapsedMs);
    assertEquals(false, new Navigator(robot, stats).turnTo(90, Config.MAX_SPEED));
    assertEquals(Config.TURN_TIMEOUT_STEPS, robot.stepsTaken());
    assertEquals(1, stats.turnTimeouts());
  }

  @Test
  void turnAcrossTheSeamGoesTheShortWay() {
    KinematicRobot robot = new KinematicRobot(Vec2.of(0, 0), 350);
    assertTrue(nav(robot).turnTo(10, Config.MAX_SPEED));
    assertTrue(Math.abs(robot.cumulativeTurnDeg) < 25, "turned " + robot.cumulativeTurnDeg);
  }

  @Test
  void alreadyAlignedTurnDoesNotMove() {
    KinematicRobot robot = new KinematicRobot(Vec2.of(0, 0), 90);
    assertTrue(nav(robot).turnTo(91, Config.MAX_SPEED));
    assertEquals(0, robot.stepsTaken());
  }

  @Test
  void driveCorrectsTheHeadingErrorTheTurnToleranceAllows() {
    KinematicRobot robot = new KinematicRobot(Vec2.of(0, 0), 2.9);
    Vec2 target = Vec2.of(530, 0);
    assertEquals(Navigator.DriveResult.ARRIVED, nav(robot).driveTo(target));
    assertTrue(robot.positionCm().distanceTo(target) < 3, "ended at " + robot.positionCm());
  }

  @Test
  void blockedDriveReportsAStallInsteadOfHanging() {
    KinematicRobot robot = new KinematicRobot(Vec2.of(0, 0), 0).withStepLimit(5_000);
    robot.blocked = p -> p.x > 100;
    MissionStats stats = new MissionStats(robot::elapsedMs);
    assertEquals(Navigator.DriveResult.STALLED, new Navigator(robot, stats).driveTo(Vec2.of(300, 0)));
    assertEquals(1, stats.driveStalls());
    assertTrue(robot.stepsTaken() < 2 * Config.STALL_WINDOW_STEPS + 100);
  }

  @Test
  void everyRouteReachesEveryWaypointInOpenSpace() {
    checkRoute(Vec2.of(-815.65, 556.873), 90, Routes.CLEARING_PATH);
    checkRoute(Routes.SEARCH_POINT, 270, Routes.TO_SEARCH_POINT);
    checkRoute(Routes.SEARCH_POINT, 120, Routes.SEARCH_TO_DROPOFF);
    checkRoute(Vec2.of(-900, 637), 0, Routes.DROPOFF_TO_SEARCH);
    checkRoute(Routes.SEARCH_POINT, 120, Routes.PLANNED_SEARCH_TO_DROPOFF);
    checkRoute(Vec2.of(-900, 637), 0, Routes.PLANNED_DROPOFF_TO_SEARCH);
    checkRoute(Routes.DROPOFF_ENTRY, 90, Routes.placementPath(4));
  }

  private static void checkRoute(Vec2 start, double heading, List<Vec2> route) {
    KinematicRobot robot = new KinematicRobot(start, heading);
    Navigator nav = nav(robot);
    for (Vec2 waypoint : route) {
      assertEquals(Navigator.DriveResult.ARRIVED, nav.goTo(waypoint));
      assertTrue(robot.positionCm().distanceTo(waypoint) < 3,
          "missed " + waypoint + ", at " + robot.positionCm());
    }
  }

  @Test
  void everyMotionRoutineLeavesTheWheelsStopped() {
    KinematicRobot robot = new KinematicRobot(Vec2.of(0, 0), 0);
    Navigator nav = nav(robot);
    nav.turnTo(120, Config.MAX_SPEED);
    assertTrue(robot.isStopped(), "after turnTo");
    nav.driveTo(Vec2.of(-100, 150));
    assertTrue(robot.isStopped(), "after driveTo");
    nav.driveSteps(Config.MANOEUVRE_SPEED, 30);
    assertTrue(robot.isStopped(), "after driveSteps");
    KinematicRobot blocked = new KinematicRobot(Vec2.of(0, 0), 0);
    blocked.blocked = p -> p.x > 50;
    nav(blocked).driveTo(Vec2.of(300, 0));
    assertTrue(blocked.isStopped(), "after a stall");
  }

  @Test
  void driveStepsMovesForwardForExactlyThatManySteps() {
    KinematicRobot robot = new KinematicRobot(Vec2.of(0, 0), 0);
    nav(robot).driveSteps(Config.MANOEUVRE_SPEED, 60);
    assertEquals(60, robot.stepsTaken());
    double expected = 60 * 0.032 * Config.MANOEUVRE_SPEED * KinematicRobot.WHEEL_RADIUS_CM;
    assertEquals(expected, robot.positionCm().x, 1e-6);
  }

  @Test
  void followRouteReportsWhetherEveryLegArrived() {
    KinematicRobot open = new KinematicRobot(Vec2.of(0, 0), 0);
    assertTrue(nav(open).followRoute(List.of(Vec2.of(100, 0), Vec2.of(100, 100))));
    KinematicRobot walled = new KinematicRobot(Vec2.of(0, 0), 0);
    walled.blocked = p -> p.x > 50;
    assertFalse(nav(walled).followRoute(List.of(Vec2.of(100, 0), Vec2.of(0, 0))));
  }

  @Test
  void goingToWhereTheRobotAlreadyIsDoesNothing() {
    KinematicRobot robot = new KinematicRobot(Vec2.of(10, 10), 45);
    assertEquals(Navigator.DriveResult.ARRIVED, nav(robot).goTo(Vec2.of(10.5, 10)));
    assertEquals(0, robot.stepsTaken());
  }

  @Test
  void headingToleranceIsInclusive() {
    KinematicRobot within = new KinematicRobot(Vec2.of(0, 0), 90);
    nav(within).turnTo(90 + Config.HEADING_TOLERANCE_DEG, Config.MAX_SPEED);
    assertEquals(0, within.stepsTaken());
    KinematicRobot outside = new KinematicRobot(Vec2.of(0, 0), 90);
    nav(outside).turnTo(90 + Config.HEADING_TOLERANCE_DEG + 0.5, Config.MAX_SPEED);
    assertTrue(outside.stepsTaken() > 0);
  }

  @Test
  void driveThatKeepsMovingButNeverArrivesTimesOut() {
    ScriptedRobot circling = new ScriptedRobot();
    circling.orbitRadiusCm = 50;
    circling.orbitDegPerStep = 2;
    MissionStats stats = new MissionStats(circling::elapsedMs);
    assertEquals(Navigator.DriveResult.TIMED_OUT,
        new Navigator(circling, stats).driveTo(Vec2.of(300, 0)));
    assertEquals(1, stats.driveTimeouts());
    assertEquals(Math.max(Config.DRIVE_TIMEOUT_MIN_STEPS, (int) (300 * Config.DRIVE_TIMEOUT_STEPS_PER_CM)),
        circling.steps);
    assertTrue(circling.isStopped());
  }

  @Test
  void turnSlowsDownNearTheTarget() {
    KinematicRobot far = new KinematicRobot(Vec2.of(0, 0), 0).withStepLimit(0);
    assertThrows(RobotPlatform.SimulationEnded.class, () -> nav(far).turnTo(90, Config.MAX_SPEED));
    assertArrayEquals(new double[] {-Config.MAX_SPEED, Config.MAX_SPEED}, far.lastCommand(), 1e-9);
    KinematicRobot near = new KinematicRobot(Vec2.of(0, 0), 0).withStepLimit(0);
    assertThrows(RobotPlatform.SimulationEnded.class, () -> nav(near).turnTo(-10, Config.MAX_SPEED));
    double speed = Config.MAX_SPEED * 10 / Config.TURN_SLOWDOWN_DEG;
    assertArrayEquals(new double[] {speed, -speed}, near.lastCommand(), 1e-9);
    KinematicRobot nearest = new KinematicRobot(Vec2.of(0, 0), 0).withStepLimit(0);
    assertThrows(RobotPlatform.SimulationEnded.class, () -> nav(nearest).turnTo(3.5, Config.MAX_SPEED));
    assertArrayEquals(new double[] {-Config.TURN_MIN_SPEED, Config.TURN_MIN_SPEED},
        nearest.lastCommand(), 1e-9);
  }

  @Test
  void headingHoldCorrectionIsProportionalToTheError() {
    KinematicRobot robot = new KinematicRobot(Vec2.of(0, 0), -2).withStepLimit(0);
    assertThrows(RobotPlatform.SimulationEnded.class, () -> nav(robot).driveTo(Vec2.of(500, 0)));
    double correction = 2 * Config.HEADING_HOLD_GAIN; // target is 2 degrees to the left
    assertArrayEquals(new double[] {Config.MAX_SPEED - correction, Config.MAX_SPEED},
        robot.lastCommand(), 1e-9);
  }

  @Test
  void stallIsCheckedEveryFortySteps() {
    KinematicRobot robot = new KinematicRobot(Vec2.of(0, 0), 0);
    robot.blocked = p -> p.x > 100; // reached after about 64 steps
    nav(robot).driveTo(Vec2.of(300, 0));
    assertEquals(3 * Config.STALL_WINDOW_STEPS, robot.stepsTaken());
  }

  @Test
  void turnThatCannotFinishStopsTheWheels() {
    KinematicRobot robot = new KinematicRobot(Vec2.of(0, 0), 0).withImperfections(0, 0.0);
    nav(robot).turnTo(90, Config.MAX_SPEED);
    assertTrue(robot.isStopped());
  }

  // Sonar watch (Config.SONAR_WATCH, off by default) and the model's sonar.

  @Test
  void sonarWatchStopsShortOfABoxInTheWay() {
    KinematicRobot robot = new KinematicRobot(Vec2.of(0, 0), 0).withBox(Vec2.of(150, 0), 15);
    MissionStats stats = new MissionStats(robot::elapsedMs);
    assertEquals(Navigator.DriveResult.OBSTACLE_AHEAD,
        new Navigator(robot, stats).driveTo(Vec2.of(300, 0), true));
    assertTrue(Double.isNaN(robot.firstContactSpeedCmS), "touched the box");
    double sonarToBox = 135 - (robot.positionCm().x + 16.4);
    assertTrue(sonarToBox > 20 && sonarToBox <= Config.SONAR_STOP_CM, "gap " + sonarToBox);
    assertEquals(1, stats.obstacleStops());
    assertTrue(robot.isStopped());
  }

  @Test
  void withoutTheSonarWatchTheSameBoxIsHitAtFullSpeed() {
    KinematicRobot robot = new KinematicRobot(Vec2.of(0, 0), 0).withBox(Vec2.of(150, 0), 15);
    assertEquals(Navigator.DriveResult.STALLED, nav(robot).driveTo(Vec2.of(300, 0)));
    assertEquals(Config.MAX_SPEED * KinematicRobot.WHEEL_RADIUS_CM, robot.firstContactSpeedCmS,
        1e-9);
  }

  @Test
  void sonarWatchSlowsDownInProportionToTheGap() {
    // Both beams start 62.5 cm from the box face: halfway between the slow and stop distances.
    double face = 16.4 + 62.5 * Math.cos(Math.toRadians(10));
    KinematicRobot robot = new KinematicRobot(Vec2.of(0, 0), 0)
        .withBox(Vec2.of(face + 50, 0), 50).withStepLimit(0);
    assertEquals(62.5, robot.frontObstacleCm(), 1e-9);
    assertThrows(RobotPlatform.SimulationEnded.class,
        () -> nav(robot).driveTo(Vec2.of(300, 0), true));
    double half = Config.MAX_SPEED * (62.5 - Config.SONAR_STOP_CM)
        / (Config.SONAR_SLOW_CM - Config.SONAR_STOP_CM);
    assertArrayEquals(new double[] {half, half}, robot.lastCommand(), 1e-9);
  }

  @Test
  void sonarWatchIgnoresObstaclesBeyondTheWaypoint() {
    KinematicRobot robot = new KinematicRobot(Vec2.of(0, 0), 0).withBox(Vec2.of(150, 0), 15);
    assertEquals(Navigator.DriveResult.ARRIVED, nav(robot).driveTo(Vec2.of(100, 0), true));
    assertEquals(100, robot.positionCm().x, 2);
  }

  @Test
  void modelSonarHearsFacingSurfacesButNotGlancingOnes() {
    // A 1 m wide box whose face is 180 cm ahead: both beams (10 deg either side) hit the face.
    KinematicRobot facing = new KinematicRobot(Vec2.of(0, 0), 0).withBox(Vec2.of(230, 0), 50);
    assertEquals(Double.POSITIVE_INFINITY,
        new KinematicRobot(Vec2.of(0, 0), 0).frontObstacleCm());
    assertEquals((180 - 16.4) / Math.cos(Math.toRadians(10)), facing.frontObstacleCm(), 1e-9);
    // A long wall parallel to the path: every beam meets it at a glancing angle, so no echo.
    KinematicRobot wall = new KinematicRobot(Vec2.of(0, 0), 0).withBox(Vec2.of(0, 1030), 1000);
    assertEquals(Double.POSITIVE_INFINITY, wall.frontObstacleCm());
  }

  /** The long carry leg from the jar bay, with a 40 cm box dropped in the middle of it. */
  private static KinematicRobot carryLegWithBoxInTheWay() throws Exception {
    WorldFile world = WorldFile.read(java.nio.file.Path.of("worlds/ProjectWorld2025.wbt"));
    Vec2 start = Vec2.of(850, 100);
    return new KinematicRobot(start, start.bearingTo(Vec2.of(350, 144)))
        .withStaticObstacles(world, KinematicRobot.ROBOT_RADIUS_CM)
        .withBox(Vec2.of(600, 122), 20);
  }

  @Test
  void aStalledLegPlansAWayAroundWhatStoppedIt() throws Exception {
    KinematicRobot robot = carryLegWithBoxInTheWay();
    MissionStats stats = new MissionStats(robot::elapsedMs);
    Vec2 target = Vec2.of(350, 144);
    assertEquals(Navigator.RouteResult.CLEAN, new Navigator(robot, stats)
        .drive(List.of(target), new Navigator.Travel().detours(true)));
    assertTrue(robot.positionCm().distanceTo(target) < 5, "ended at " + robot.positionCm());
    assertEquals(1, stats.driveStalls());
    assertEquals(1, stats.detours());
  }

  @Test
  void withoutDetoursAStalledLegIsSkipped() throws Exception {
    KinematicRobot robot = carryLegWithBoxInTheWay();
    MissionStats stats = new MissionStats(robot::elapsedMs);
    assertEquals(Navigator.RouteResult.WITH_PROBLEMS, new Navigator(robot, stats)
        .drive(List.of(Vec2.of(350, 144)), new Navigator.Travel()));
    assertTrue(robot.positionCm().x > 600, "stopped at the box, at " + robot.positionCm());
    assertEquals(0, stats.detours());
  }

  @Test
  void aRouteIsAbandonedAsSoonAsItsConditionFails() {
    KinematicRobot robot = new KinematicRobot(Vec2.of(0, 0), 0);
    int[] checks = {0};
    Navigator.RouteResult result = nav(robot).drive(List.of(Vec2.of(300, 0), Vec2.of(300, 300)),
        new Navigator.Travel().keepGoingWhile(() -> ++checks[0] < 50));
    assertEquals(Navigator.RouteResult.ABORTED, result);
    assertTrue(robot.isStopped());
    assertTrue(robot.positionCm().x < 300 && robot.positionCm().y == 0, "at " + robot.positionCm());
  }

  @Test
  void aRouteFromAnywhereIsPlannedAroundTheWalls() throws Exception {
    // From the drop-off end of the carry route back to the jar bay: a straight line would cross
    // wooden boxes; the planned one doesn't, so it arrives without stalling.
    WorldFile world = WorldFile.read(java.nio.file.Path.of("worlds/ProjectWorld2025.wbt"));
    KinematicRobot robot = new KinematicRobot(Vec2.of(-590, 450), 0)
        .withStaticObstacles(world, KinematicRobot.ROBOT_RADIUS_CM).withStepLimit(100_000);
    MissionStats stats = new MissionStats(robot::elapsedMs);
    assertEquals(Navigator.RouteResult.CLEAN,
        new Navigator(robot, stats).driveViaPlan(Vec2.of(850, 100), new Navigator.Travel()));
    assertTrue(robot.positionCm().distanceTo(Vec2.of(850, 100)) < 5);
    assertEquals(0, stats.navigationProblems());
  }

  @Test
  void aSonarStopShortOfABoxIsFollowedByADetour() throws Exception {
    KinematicRobot robot = carryLegWithBoxInTheWay();
    MissionStats stats = new MissionStats(robot::elapsedMs);
    Vec2 target = Vec2.of(350, 144);
    assertEquals(Navigator.RouteResult.CLEAN, new Navigator(robot, stats)
        .drive(List.of(target), new Navigator.Travel().watchAhead(true).detours(true)));
    assertTrue(robot.positionCm().distanceTo(target) < 5, "ended at " + robot.positionCm());
    assertEquals(1, stats.obstacleStops());
    assertEquals(0, stats.driveStalls()); // it never touched the box
    assertEquals(1, stats.detours());
    assertTrue(Double.isNaN(robot.firstContactSpeedCmS), "touched the box");
  }

  @Test
  void theAssumedObstacleIsASquareCentredAheadOfTheRobot() {
    Vec2[] north = Navigator.squareAhead(Vec2.of(100, 50), 90, 35, 40);
    assertArrayEquals(new Vec2[] {Vec2.of(80, 65), Vec2.of(120, 65), Vec2.of(120, 105),
        Vec2.of(80, 105)}, north);
    Vec2[] east = Navigator.squareAhead(Vec2.of(0, 0), 0, 50, 20);
    assertTrue(east[0].distanceTo(Vec2.of(40, -10)) < 1e-9 && east[2].distanceTo(Vec2.of(60, 10)) < 1e-9,
        java.util.Arrays.toString(east));
  }

  @Test
  void aRouteIsAbandonedMidTurnWhenItsConditionFails() {
    KinematicRobot robot = new KinematicRobot(Vec2.of(0, 0), 0);
    int[] checks = {0};
    assertEquals(Navigator.DriveResult.ABORTED, nav(robot).goTo(Vec2.of(-300, 0),
        new Navigator.Travel().keepGoingWhile(() -> ++checks[0] < 5)));
    assertTrue(robot.isStopped());
    assertTrue(Math.abs(robot.cumulativeTurnDeg) > 1 && Math.abs(robot.cumulativeTurnDeg) < 90,
        "turned " + robot.cumulativeTurnDeg);
    assertTrue(robot.positionCm().distanceTo(Vec2.of(0, 0)) < 1e-9, "drove " + robot.positionCm());
  }

  @Test
  void aRouteIsAbandonedMidDriveWhenItsConditionFails() {
    KinematicRobot robot = new KinematicRobot(Vec2.of(0, 0), 0);
    int[] checks = {0};
    assertEquals(Navigator.DriveResult.ABORTED, nav(robot).goTo(Vec2.of(300, 0),
        new Navigator.Travel().keepGoingWhile(() -> ++checks[0] < 20)));
    assertTrue(robot.isStopped());
    assertTrue(robot.positionCm().x > 10 && robot.positionCm().x < 300, "at " + robot.positionCm());
  }

  @Test
  void aPlannedRouteStartsAndEndsAtPointsTooCloseToAWallToPlanFrom() {
    // (-600, 965) and (-300, 965) are 17 cm from the north wall, inside the planner's 30 cm
    // clearance: the route steps out to open floor first and back in at the end.
    KinematicRobot robot = new KinematicRobot(Vec2.of(-600, 965), 0);
    Vec2 target = Vec2.of(-300, 965);
    assertEquals(Navigator.RouteResult.CLEAN, nav(robot).driveViaPlan(target, new Navigator.Travel()));
    assertTrue(robot.positionCm().distanceTo(target) < 5, "ended at " + robot.positionCm());
  }
}
