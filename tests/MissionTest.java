import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/** End-to-end runs of the v2 mission logic on the kinematic model (control flow, not physics). */
class MissionTest {

  private static WorldFile world;

  @BeforeAll
  static void setUp() throws Exception {
    Log.useSink(line -> { });
    world = WorldFile.read(Path.of("worlds/ProjectWorld2025.wbt"));
  }

  /** Walls and immovable boxes are solid, so a route through them stalls the robot. */
  private static KinematicRobot robotWithJars() {
    return KinematicRobot.atMissionStart(world);
  }

  @Test
  void deliversAllFiveJarsToTheirPads() {
    KinematicRobot robot = robotWithJars();
    Mission mission = new Mission(robot);
    assertEquals(5, mission.run());
    assertEquals(5, robot.deliveredJars.size());
    for (Vec2 jar : robot.deliveredJars) {
      assertTrue(jar.x < -940 && jar.y > 590 && jar.y < 740, "jar left at " + jar);
    }
    MissionStats stats = mission.stats();
    assertEquals(0, stats.navigationProblems());
    assertEquals(0, stats.failedSearches());
  }

  @Test
  void stillDeliversAllJarsWithMotorLagAndWheelSlip() {
    KinematicRobot robot = robotWithJars().withImperfections(2, 0.7);
    assertEquals(5, new Mission(robot).run());
  }

  @Test
  void optInPlannedRoutesAlsoDeliverAllJarsAndFinishSooner() {
    KinematicRobot handTuned = robotWithJars();
    assertEquals(5, new Mission(handTuned, Routes.HAND_TUNED).run());
    KinematicRobot planned = robotWithJars();
    assertEquals(5, new Mission(planned, Routes.PLANNED).run());
    assertTrue(planned.elapsedMs() < handTuned.elapsedMs(),
        planned.elapsedMs() + " ms vs " + handTuned.elapsedMs() + " ms");
  }

  @Test
  void identicalRunsGiveIdenticalReports() {
    List<String> logs = new ArrayList<>();
    for (int run = 0; run < 2; run++) {
      StringBuilder log = new StringBuilder();
      Log.useSink(line -> log.append(line).append('\n'));
      new Mission(robotWithJars()).run();
      logs.add(log.toString());
    }
    Log.useSink(line -> { });
    assertEquals(logs.get(0), logs.get(1));
    // The run ends by logging the result and the report.
    String log = logs.get(0);
    assertTrue(log.contains("ALL JARS COLLECTED!"), log);
    assertTrue(log.indexOf("MISSION REPORT") > log.indexOf("ALL JARS COLLECTED!"));
    assertTrue(log.contains("5/5 jars delivered"));
  }

  @Test
  void stopsAfterRepeatedFailedSearchesInsteadOfLoopingForever() {
    KinematicRobot robot = new KinematicRobot(world.robotStart, 90).withStepLimit(1_000_000);
    Mission mission = new Mission(robot);
    assertEquals(0, mission.run());
    assertEquals(Config.MAX_CONSECUTIVE_FAILED_SEARCHES, mission.stats().failedSearches());
    assertTrue(mission.stats().report().contains("0/5 jars delivered"));
    assertTrue(mission.stats().jars().stream().noneMatch(jar -> jar.delivered));
  }

  @Test
  void unwindsCleanlyWhenWebotsEndsTheSimulation() {
    KinematicRobot robot = robotWithJars().withStepLimit(3_000);
    assertThrows(RobotPlatform.SimulationEnded.class, () -> new Mission(robot).run());
    assertEquals(3_000, robot.stepsTaken());
  }

  /**
   * Regression test for v2's most important fix: after a failed search the robot must go back to
   * the search point the short way. Taking the drop-off route instead (v1's behaviour) drives
   * straight through immovable boxes, which stalls this model.
   */
  @Test
  void failedSearchesRetryFromTheSearchPointWithoutHittingAnything() {
    KinematicRobot robot = new KinematicRobot(world.robotStart, 90)
        .withStaticObstacles(world, KinematicRobot.ROBOT_RADIUS_CM);
    robot.jars.add(world.ofKind(WorldFile.Kind.HONEY_JAR).get(1).center); // only one jar
    Mission mission = new Mission(robot);
    assertEquals(1, mission.run());
    assertEquals(Config.MAX_CONSECUTIVE_FAILED_SEARCHES, mission.stats().failedSearches());
    assertEquals(0, mission.stats().driveStalls() + mission.stats().driveTimeouts());
  }

  @Test
  void startUpOpensTheGripperAndWaitsLikeV1BeforeMoving() {
    KinematicRobot robot = robotWithJars().withStepLimit(100);
    robot.fingers = Config.FINGERS_CLOSED; // start-up must open and lower the gripper
    robot.lift = Config.LIFT_UP;
    assertThrows(RobotPlatform.SimulationEnded.class, () -> new Mission(robot).run());
    // v1: delay(1000) = 32 steps of 32 ms, then a 10-step sensor test.
    assertEquals(32 + Config.SENSOR_SELF_TEST_STEPS, robot.firstMoveStep);
    assertEquals(Config.FINGERS_OPEN, robot.fingers);
    assertEquals(Config.LIFT_DOWN, robot.lift);
  }

  @Test
  void reportCoversEveryPhaseAndJar() {
    Mission mission = new Mission(robotWithJars());
    mission.run();
    MissionStats stats = mission.stats();
    assertEquals(List.of(Mission.PHASE_STARTUP, Mission.PHASE_CLEARING, Mission.PHASE_TO_SEARCH,
        Mission.PHASE_SEARCH, Mission.PHASE_TO_DROPOFF, Mission.PHASE_PLACE),
        new ArrayList<>(stats.phases().keySet()));
    for (long ms : stats.phases().values()) {
      assertTrue(ms > 0);
    }
    assertEquals(5, stats.jars().size());
    for (int i = 0; i < 5; i++) {
      assertEquals(i + 1, stats.jars().get(i).number);
    }
    for (MissionStats.JarRecord jar : stats.jars()) {
      assertTrue(jar.delivered && jar.scans >= 1 && jar.approaches >= 1 && jar.endMs > jar.startMs);
    }
    assertTrue(stats.report().contains("5/5 jars delivered"));
  }

  @Test
  void phaseTimesAddUpToTheWholeMission() {
    KinematicRobot robot = robotWithJars();
    Mission mission = new Mission(robot);
    mission.run();
    long total = 0;
    for (long ms : mission.stats().phases().values()) {
      total += ms;
    }
    assertEquals(robot.elapsedMs(), total);
  }

  @Test
  void grabCheckRecoversFromJarsThatSlipOut() {
    KinematicRobot robot = robotWithJars();
    robot.grabsThatMiss = 2;
    Mission mission = new Mission(robot, new Mission.Options().verifyGrabs(true));
    assertEquals(5, mission.run());
    assertEquals(5, robot.deliveredJars.size());
    assertEquals(2, mission.stats().missedGrabs());
  }

  @Test
  void withoutTheGrabCheckASlippedJarIsCountedButNeverDelivered() {
    KinematicRobot robot = robotWithJars();
    robot.grabsThatMiss = 1;
    assertEquals(5, new Mission(robot).run()); // what the robot believes
    assertEquals(4, robot.deliveredJars.size()); // what is actually on the pads
    assertEquals(1, robot.jars.size()); // still on the floor
  }

  @Test
  void deliversAllJarsOnTheExtremeRobotVariants() {
    for (int lag : new int[] {0, 3}) {
      for (double turnRate : new double[] {0.6, 1.0}) {
        KinematicRobot robot = robotWithJars().withImperfections(lag, turnRate);
        assertEquals(5, new Mission(robot).run(), lag + " / " + turnRate);
        assertEquals(5, robot.deliveredJars.size());
      }
    }
  }
}
