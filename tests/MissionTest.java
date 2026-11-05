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

  private static KinematicRobot robotWithJars() {
    // Facing +y (90 deg): the robot's rotation in the world file is the identity, and the
    // Pioneer's forward axis is -Z, which is +y in the controllers' frame.
    KinematicRobot robot = new KinematicRobot(world.robotStart, 90);
    for (WorldFile.Obstacle jar : world.ofKind(WorldFile.Kind.HONEY_JAR)) {
      robot.jars.add(jar.center);
    }
    return robot;
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
    assertEquals(0, stats.driveStalls + stats.driveTimeouts + stats.turnTimeouts);
    assertEquals(0, stats.failedSearches);
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
  }

  @Test
  void stopsAfterRepeatedFailedSearchesInsteadOfLoopingForever() {
    KinematicRobot robot = new KinematicRobot(world.robotStart, 90).withStepLimit(1_000_000);
    Mission mission = new Mission(robot);
    assertEquals(0, mission.run());
    assertEquals(Config.MAX_CONSECUTIVE_FAILED_SEARCHES, mission.stats().failedSearches);
    assertTrue(mission.stats().report().contains("0/5 jars delivered"));
  }

  @Test
  void unwindsCleanlyWhenWebotsEndsTheSimulation() {
    KinematicRobot robot = robotWithJars().withStepLimit(3_000);
    assertThrows(RobotPlatform.SimulationEnded.class, () -> new Mission(robot).run());
    assertEquals(3_000, robot.stepsTaken());
  }
}
