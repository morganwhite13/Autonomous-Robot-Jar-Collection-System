import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

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
}
