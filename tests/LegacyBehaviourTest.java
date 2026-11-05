import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.nio.file.Path;
import org.junit.jupiter.api.Test;

/**
 * Characterisation tests: they pin down the v1 behaviours that docs/BEFORE_AND_AFTER.md
 * describes, using the ports in {@link LegacyV1}. If one of these fails, the documentation
 * about v1 is out of date.
 */
class LegacyBehaviourTest {

  @Test
  void v1ScanRotatesAboutThreeQuartersOfATurn() {
    KinematicRobot robot = new KinematicRobot(Vec2.of(0, 0), 0);
    LegacyV1.rotateAndFindGreen(robot);
    assertEquals(271, robot.cumulativeTurnDeg, 2);
  }

  @Test
  void v1ScanCoversEvenLessWhenTheWheelsSlip() {
    KinematicRobot robot = new KinematicRobot(Vec2.of(0, 0), 0).withImperfections(2, 0.7);
    LegacyV1.rotateAndFindGreen(robot);
    assertTrue(robot.cumulativeTurnDeg < 200, "turned " + robot.cumulativeTurnDeg);
  }

  @Test
  void v1ScanMissesAJarJustRightOfItsStartingHeading() {
    KinematicRobot robot = new KinematicRobot(Vec2.of(0, 0), 0);
    double rad = Math.toRadians(-45);
    robot.jars.add(Vec2.of(100 * Math.cos(rad), 100 * Math.sin(rad)));
    assertNull(LegacyV1.rotateAndFindGreen(robot));
  }

  @Test
  void v1OpenLoopDriveDriftsFromTheWaypoint() {
    KinematicRobot robot = new KinematicRobot(Vec2.of(0, 0), 2.9);
    LegacyV1.moveHardcodedAhead(robot, 0, 0, 530, 0, 10_000);
    assertTrue(robot.positionCm().distanceTo(Vec2.of(530, 0)) > 25);
  }

  @Test
  void v1DriveNeverReturnsWhenBlocked() {
    KinematicRobot robot = new KinematicRobot(Vec2.of(0, 0), 0);
    robot.blocked = p -> p.x > 100;
    assertEquals(-1, LegacyV1.moveHardcodedAhead(robot, 0, 0, 300, 0, 20_000));
  }

  @Test
  void v1TurnCanBounceAcrossItsAcceptanceWindowForever() {
    int neverFinished = 0;
    for (int start = 0; start < 360; start += 3) {
      for (int target = 0; target < 360; target += 7) {
        KinematicRobot robot = new KinematicRobot(Vec2.of(0, 0), start);
        double rad = Math.toRadians(target);
        if (LegacyV1.makeHardcodedTurn(robot, 0, 0, Math.cos(rad), Math.sin(rad), 1000) < 0) {
          neverFinished++;
        }
      }
    }
    assertTrue(neverFinished > 0);
  }

  @Test
  void v1ApproachTimeLimitCoversUnderSixtyCentimetres() {
    KinematicRobot robot = new KinematicRobot(Vec2.of(0, 0), 0);
    robot.drive(LegacyV1.APPROACH_SPEED, LegacyV1.APPROACH_SPEED);
    robot.waitSteps(LegacyV1.APPROACH_EFFECTIVE_STEPS);
    assertEquals(58.5, robot.positionCm().x, 0.5);
  }

  // Whole-mission behaviour (tests/LegacyV1Mission.java), as described in benchmark-results.md.

  private static KinematicRobot missionRobot(WorldFile world) {
    KinematicRobot robot = new KinematicRobot(world.robotStart, 90).withStaticObstacles(world, 15);
    for (WorldFile.Obstacle jar : world.ofKind(WorldFile.Kind.HONEY_JAR)) {
      robot.jars.add(jar.center);
    }
    return robot;
  }

  @Test
  void v1MissionNeverGetsPastTheClearingRouteOnTheIdealModel() throws Exception {
    WorldFile world = WorldFile.read(Path.of("worlds/ProjectWorld2025.wbt"));
    KinematicRobot robot = missionRobot(world).withStepLimit(20_000);
    LegacyV1Mission.Result result = new LegacyV1Mission(robot, 1).run();
    assertFalse(result.finished);
    assertEquals(0, result.delivered);
    assertTrue(result.activity.startsWith("turning towards (-566, 375) (leg 5 of 15"),
        result.activity);
    assertTrue(Angles.within(robot.headingDeg(), 0, 6), "heading " + robot.headingDeg());
  }

  @Test
  void v1NeverFinishesATurnToZeroDegreesAtFullSpeed() {
    // Leg 5 of the clearing route on its own: from facing +y (90 deg) to bearing 0 deg.
    KinematicRobot robot = new KinematicRobot(Vec2.of(-820, 375), 90);
    assertEquals(-1, LegacyV1.makeHardcodedTurn(robot, -820, 375, -566, 375, 10_000));
    assertTrue(Angles.within(robot.headingDeg(), 0, 6), "heading " + robot.headingDeg());
    // v2's turn to the same heading finishes.
    KinematicRobot v2 = new KinematicRobot(Vec2.of(-820, 375), 90);
    assertTrue(new Navigator(v2, new MissionStats(v2::elapsedMs)).turnTo(0, Config.MAX_SPEED));
  }

  @Test
  void v1MissionWithLagAndSlipDeliversTwoJarsThenPushesAgainstABox() throws Exception {
    WorldFile world = WorldFile.read(Path.of("worlds/ProjectWorld2025.wbt"));
    KinematicRobot robot = missionRobot(world).withImperfections(2, 0.7).withStepLimit(30_000);
    LegacyV1Mission.Result result = new LegacyV1Mission(robot, 1).run();
    assertFalse(result.finished);
    assertEquals(2, result.delivered);
    assertTrue(result.activity.contains("`navigateFromDropoffToSearch`"), result.activity);
    WorldFile.Obstacle box = world.staticObstacles().stream()
        .filter(o -> o.name.equals("wooden box(5)")).findFirst().orElseThrow();
    assertTrue(RoutePlanner.distanceToPolygon(robot.positionCm(), box.corners) < 16);
    // Stuck long before the run was stopped: no stall detection.
    assertTrue(result.activitySinceMs < robot.elapsedMs() / 2);
  }
}
