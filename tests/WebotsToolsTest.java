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
  }

  private void writeRun(String name, String result, String log) throws Exception {
    Path dir = Files.createDirectories(tmp.resolve(name));
    Files.write(dir.resolve("result.properties"), result.getBytes(StandardCharsets.UTF_8));
    Files.write(dir.resolve("webots.log"), log.getBytes(StandardCharsets.UTF_8));
    Files.write(dir.resolve("trajectory.csv"), "t,x,y\n0,0,0\n1,60,80\n".getBytes(StandardCharsets.UTF_8));
  }
}
