import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

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
}
