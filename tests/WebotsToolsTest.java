import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

/** Tests for the pure parts of the Webots comparison (tools/run-in-webots.sh). */
class WebotsToolsTest {

  @TempDir
  Path tmp;

  @Test
  void jarCountsAsDeliveredOnlyAfterRestingInTheLane() {
    DeliveryTracker tracker = new DeliveryTracker(1);
    // Carried through the lane: keeps moving, so not delivered.
    for (int i = 0; i <= 40; i++) {
      tracker.update(0, i * 0.1, -900 - i, 607);
    }
    assertEquals(0, tracker.delivered());
    // Released at x = -960 and left alone.
    for (int i = 0; i <= 60; i++) {
      tracker.update(0, 10 + i * 0.1, -960, 607);
    }
    assertEquals(1, tracker.delivered());
    assertEquals(10.0, tracker.lastDeliveryTime(), 1e-9);
  }

  @Test
  void jarRestingOutsideTheLaneIsNotDelivered() {
    DeliveryTracker tracker = new DeliveryTracker(1);
    for (int i = 0; i <= 100; i++) {
      tracker.update(0, i * 0.1, 903, -25); // still in the jar bay
    }
    assertEquals(0, tracker.delivered());
  }

  @Test
  void knockingADeliveredJarAwayUndoesTheDelivery() {
    DeliveryTracker tracker = new DeliveryTracker(2);
    for (int i = 0; i <= 60; i++) {
      tracker.update(0, i * 0.1, -960, 607);
      tracker.update(1, i * 0.1, -960, 637);
    }
    assertTrue(tracker.allDelivered());
    tracker.update(1, 7, -880, 637);
    assertEquals(1, tracker.delivered());
    assertEquals(1, tracker.disturbances());
    assertTrue(tracker.toProperties(7, false).contains("disturbances=1"));
  }

  @Test
  void reportComparesRunsFromTheirOutputFiles() throws Exception {
    writeRun("v1", "jars=5\ndelivered=5\ndeliveryTimesS=100.00,700.00\nsimTimeS=702.5\n",
        "[ProjectController3] WARNING: Approach taking too long, probably hitting wall!\n"
            + "[ProjectController3] Lost green for too long! Aborting approach.\n"
            + "Jars remaining: 4\nJars remaining: 3\n");
    writeRun("v2", "jars=5\ndelivered=3\ndeliveryTimesS=90.00,200.00,300.00\nsimTimeS=3600\n",
        "[JarCollectorV2] [  50.0s] WARNING: stalled at (1, 2) while driving to (3, 4)\n");
    String md = WebotsReport.markdown(List.of(
        WebotsReport.load(tmp.resolve("v1"), "v1"), WebotsReport.load(tmp.resolve("v2"), "v2")));
    assertTrue(md.contains("| Jars delivered | 5/5 | 3/5 |"), md);
    assertTrue(md.contains("| Mission time (last jar delivered) | 700 s (11.7 min) | did not finish"), md);
    assertTrue(md.contains("| Approach aborted | 2 | 0 |"), md);
    assertTrue(md.contains("| Drive stalled or timed out (v2 detects these; v1 cannot) | 0 | 1 |"), md);
    assertTrue(md.contains("| Distance driven | 1 m | 1 m |"), md);
    assertTrue(md.contains("| Jars the controller says it delivered | 2 | 0 |"), md);
    assertTrue(md.contains("| Stuck in one place from | - | - |"), md);
  }

  @Test
  void reportShowsWhenARunThatNeverFinishedStoppedMoving() throws Exception {
    writeRun("v1", "jars=5\ndelivered=0\nsimTimeS=600\ntimeLimitHit=true\n", "");
    Files.write(tmp.resolve("v1").resolve("trajectory.csv"),
        "t,x,y\n0,0,0\n100,300,0\n200,500,0\n300,505,0\n600,507,0\n".getBytes(StandardCharsets.UTF_8));
    writeRun("v2", "jars=5\ndelivered=0\nsimTimeS=600\ntimeLimitHit=true\n", "");
    Files.write(tmp.resolve("v2").resolve("trajectory.csv"),
        "t,x,y\n0,0,0\n500,0,0\n550,300,0\n600,300,5\n".getBytes(StandardCharsets.UTF_8));
    WebotsReport.Run stuck = WebotsReport.load(tmp.resolve("v1"), "v1");
    assertEquals(200, stuck.stuckFromS(), 1e-9); // within 10 cm of the end from 200 s
    assertTrue(Double.isNaN(WebotsReport.load(tmp.resolve("v2"), "v2").stuckFromS()),
        "moved until 550 s: less than two minutes at rest");
    writeRun("v2done", "jars=5\ndelivered=4\nsimTimeS=600\ntimeLimitHit=true\n",
        "=== MISSION REPORT (v2) ===\n");
    Files.write(tmp.resolve("v2done").resolve("trajectory.csv"),
        "t,x,y\n0,0,0\n100,300,0\n600,300,0\n".getBytes(StandardCharsets.UTF_8));
    assertTrue(Double.isNaN(WebotsReport.load(tmp.resolve("v2done"), "v2done").stuckFromS()),
        "parked after finishing its mission: not stuck");
  }

  @Test
  void pathsMapDrawsOneLinePerRun() throws Exception {
    WorldFile world = WorldFile.read(Path.of("worlds/ProjectWorld2025.wbt"));
    Map<String, List<Vec2>> paths = new LinkedHashMap<>();
    paths.put("v1 (before)", List.of(Vec2.of(0, 0), Vec2.of(100, 0)));
    paths.put("v2 (after)", List.of(Vec2.of(0, 0), Vec2.of(0, 100)));
    String svg = new MapRenderer().renderPaths(world, paths);
    assertEquals(2, svg.split("<polyline", -1).length - 1);
    assertTrue(svg.contains("v1 (before)") && svg.contains("v2 (after)"));
    assertFalse(svg.contains("arrow-clear"), "planned routes are not drawn");
    // A dot where each run ended, so a run that got stuck shows where.
    assertTrue(svg.contains(String.format(java.util.Locale.ROOT, "cx=\"%.1f\" cy=\"%.1f\" r=\"16\"",
        MapRenderer.sx(100), MapRenderer.sy(0))), "v1 end");
    assertEquals(2, svg.split("r=\"16\"", -1).length - 1);
    assertTrue(svg.contains("Where the run ended"));
  }

  private void writeRun(String name, String result, String log) throws Exception {
    Path dir = Files.createDirectories(tmp.resolve(name));
    Files.write(dir.resolve("result.properties"), result.getBytes(StandardCharsets.UTF_8));
    Files.write(dir.resolve("webots.log"), log.getBytes(StandardCharsets.UTF_8));
    Files.write(dir.resolve("trajectory.csv"), "t,x,y\n0,0,0\n1,60,80\n".getBytes(StandardCharsets.UTF_8));
  }

  @Test
  void shuffledWorldsPutTheJarsInTheBayAndBarelyMoveTheRobot() throws Exception {
    WorldFile world = WorldFile.read(Path.of("worlds/ProjectWorld2025.wbt"));
    for (long seed = 1; seed <= 25; seed++) {
      WorldShuffle.Variant v = WorldShuffle.variant(world, seed);
      assertEquals(5, v.jars.size());
      for (int i = 0; i < v.jars.size(); i++) {
        Vec2 jar = v.jars.get(i);
        assertTrue(jar.x >= WorldShuffle.JAR_MIN_X && jar.x <= WorldShuffle.JAR_MAX_X
            && jar.y >= WorldShuffle.JAR_MIN_Y && jar.y <= WorldShuffle.JAR_MAX_Y, "seed " + seed);
        for (int k = 0; k < i; k++) {
          assertTrue(jar.distanceTo(v.jars.get(k)) >= WorldShuffle.JAR_SPACING_CM, "seed " + seed);
        }
        for (WorldFile.Obstacle o : world.obstacles) {
          if (o.kind != WorldFile.Kind.HONEY_JAR) {
            assertTrue(RoutePlanner.distanceToPolygon(jar, o.corners) >= WorldShuffle.JAR_CLEARANCE_CM,
                "seed " + seed + " jar " + jar + " vs " + o.name);
          }
        }
      }
      assertTrue(v.start.distanceTo(world.robotStart) <= WorldShuffle.START_SHIFT_CM);
      assertTrue(Math.abs(v.startTurnDeg) <= WorldShuffle.START_TURN_DEG);
    }
    assertEquals(WorldShuffle.variant(world, 7).jars, WorldShuffle.variant(world, 7).jars);
  }

  @Test
  void shufflingRewritesOnlyTheRobotPoseAndTheJarPositions() throws Exception {
    Path copy = tmp.resolve("w.wbt");
    Files.copy(Path.of("worlds/ProjectWorld2025.wbt"), copy);
    WorldShuffle.main(new String[] {copy.toString(), "3"});
    List<String> before = Files.readAllLines(Path.of("worlds/ProjectWorld2025.wbt"));
    List<String> after = Files.readAllLines(copy);
    assertEquals(before.size(), after.size());
    int changed = 0;
    for (int i = 0; i < before.size(); i++) {
      if (!before.get(i).equals(after.get(i))) {
        changed++;
        String t = after.get(i).trim();
        assertTrue(t.startsWith("translation ") || t.startsWith("rotation 0 1 0 "), after.get(i));
      }
    }
    assertEquals(7, changed); // robot translation and rotation, five jars
    WorldFile shuffled = WorldFile.read(copy);
    WorldShuffle.Variant v = WorldShuffle.variant(WorldFile.read(Path.of("worlds/ProjectWorld2025.wbt")), 3);
    assertTrue(shuffled.robotStart.distanceTo(v.start) < 0.01);
    List<WorldFile.Obstacle> jars = shuffled.ofKind(WorldFile.Kind.HONEY_JAR);
    for (int i = 0; i < 5; i++) {
      assertTrue(jars.get(i).center.distanceTo(v.jars.get(i)) < 0.01);
    }
    Path untouched = tmp.resolve("same.wbt");
    Files.copy(Path.of("worlds/ProjectWorld2025.wbt"), untouched);
    WorldShuffle.main(new String[] {untouched.toString(), "0"});
    assertEquals(before, Files.readAllLines(untouched));
  }

  @Test
  void sweepReportCountsWorldsWhereEveryJarWasDelivered() throws Exception {
    writeRun("v2", "jars=5\ndelivered=5\ndeliveryTimesS=100,800.00\nsimTimeS=802\n",
        "Jars remaining: 4\nJars remaining: 3\nJars remaining: 2\nJars remaining: 1\nJars remaining: 0\n");
    WebotsReport.Run full = WebotsReport.load(tmp.resolve("v2"), "v2");
    writeRun("v1", "jars=5\ndelivered=0\nsimTimeS=1500\ntimeLimitHit=true\n", "Jars remaining: 4\n");
    WebotsReport.Run none = WebotsReport.load(tmp.resolve("v1"), "v1");
    String md = WebotsSweepReport.markdown(List.of(1, 2), List.of("v1", "v2"),
        List.of(java.util.Arrays.asList(none, full), java.util.Arrays.asList(none, null)), "1500");
    assertTrue(md.contains("| Delivered all 5 jars | 0 of 2 worlds | 1 of 1 worlds |"), md);
    assertTrue(md.contains("| Jars delivered in total | 0 of 10 | 5 of 5 |"), md);
    assertTrue(md.contains("| Jars the controller says it delivered | 2 | 5 |"), md);
    assertTrue(md.contains("| Mission time when all 5 were delivered (median, range) | - | 800 s (800-800 s) |"), md);
    assertTrue(md.contains("| 1 | 0/5 (claimed 1) | 5/5, 800 s |"), md);
    assertTrue(md.contains("| 2 | 0/5 (claimed 1) | not run |"), md);
  }
}
