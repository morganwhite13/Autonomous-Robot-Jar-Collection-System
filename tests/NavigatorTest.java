import static org.junit.jupiter.api.Assertions.assertEquals;
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
    assertEquals(1, stats.driveStalls);
    assertTrue(robot.stepsTaken() < 2 * Config.STALL_WINDOW_STEPS + 100);
  }

  @Test
  void everyRouteReachesEveryWaypointInOpenSpace() {
    checkRoute(Vec2.of(-815.65, 556.873), 0, Routes.CLEARING_PATH);
    checkRoute(Routes.SEARCH_POINT, 270, Routes.TO_SEARCH_POINT);
    checkRoute(Routes.SEARCH_POINT, 120, Routes.SEARCH_TO_DROPOFF);
    checkRoute(Vec2.of(-900, 637), 0, Routes.DROPOFF_TO_SEARCH);
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
}
