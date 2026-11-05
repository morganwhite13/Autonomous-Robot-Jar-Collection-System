import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class MissionStatsTest {

  private long now;

  @Test
  void accumulatesRepeatedPhasesAndTracksJars() {
    MissionStats stats = new MissionStats(() -> now);
    stats.beginPhase("Search");
    stats.beginJar(1);
    stats.recordScan();
    now = 2_000;
    stats.beginPhase("Carry");
    now = 5_000;
    stats.jarDelivered();
    stats.beginJar(2);
    stats.beginPhase("Search");
    now = 6_500;
    stats.recordScan();
    stats.recordScan();
    stats.endPhase();

    assertEquals(3_500L, stats.phases().get("Search"));
    assertEquals(3_000L, stats.phases().get("Carry"));
    assertEquals(1, stats.jarsDelivered());
    assertEquals(1, stats.jars().get(0).scans);
    assertEquals(2, stats.jars().get(1).scans);
  }

  @Test
  void approachesAreCountedPerJar() {
    MissionStats stats = new MissionStats(() -> now);
    stats.recordApproach(); // no open jar yet: ignored
    stats.beginJar(1);
    stats.recordApproach();
    stats.recordApproach();
    assertEquals(2, stats.jars().get(0).approaches);
    assertTrue(stats.report().contains("not delivered"), stats.report());
  }

  @Test
  void beginJarKeepsTheOpenRecordForRetries() {
    MissionStats stats = new MissionStats(() -> now);
    stats.beginJar(3);
    stats.recordScan();
    stats.beginJar(3);
    stats.recordScan();
    assertEquals(1, stats.jars().size());
    assertEquals(2, stats.jars().get(0).scans);
  }

  @Test
  void reportShowsResultAndAnomalies() {
    MissionStats stats = new MissionStats(() -> now);
    stats.beginJar(1);
    now = 12_300;
    stats.jarDelivered();
    stats.recordDriveStall();
    stats.recordDriveStall();
    String report = stats.report();
    assertTrue(report.contains("1/5 jars delivered in 12.3 s"), report);
    assertTrue(report.contains("drive stalls=2"), report);
  }

  @Test
  void reportLinesShowExactTimesAndCounts() {
    MissionStats stats = new MissionStats(() -> now);
    stats.beginPhase("Search");
    stats.beginJar(1);
    stats.recordScan();
    stats.recordApproach();
    now = 12_300;
    stats.jarDelivered();
    stats.beginJar(2);
    now = 15_000;
    String report = stats.report();
    assertTrue(report.contains("  Search                       15.0 s"), report);
    assertTrue(report.contains("  1       12.3      1           1  delivered"), report);
    assertTrue(report.contains("  2        2.7      0           0  not delivered"), report);
  }

  @Test
  void aJarDeliveredAtTimeZeroIsStillFinished() {
    MissionStats stats = new MissionStats(() -> now);
    now = 0;
    stats.beginJar(1);
    stats.jarDelivered();
    stats.beginJar(1); // same number again after delivery: a new record
    assertEquals(2, stats.jars().size());
  }

  @Test
  void aJarDeliveredAtTimeZeroTakesNoTimeInTheReport() {
    MissionStats stats = new MissionStats(() -> now);
    now = 0;
    stats.beginJar(1);
    stats.jarDelivered();
    now = 5_000;
    assertTrue(stats.report().contains("  1        0.0      0           0  delivered"),
        stats.report());
  }

  @Test
  void navigationProblemsAddUpStallsAndTimeouts() {
    MissionStats stats = new MissionStats(() -> now);
    stats.recordDriveStall();
    stats.recordDriveTimeout();
    stats.recordDriveTimeout();
    stats.recordTurnTimeout();
    stats.recordTurnTimeout();
    stats.recordTurnTimeout();
    stats.recordFailedSearch();
    stats.recordObstacleStop();
    stats.recordMissedGrab();
    assertEquals(1, stats.driveStalls());
    assertEquals(2, stats.driveTimeouts());
    assertEquals(3, stats.turnTimeouts());
    assertEquals(1, stats.obstacleStops());
    assertEquals(7, stats.navigationProblems());
    assertEquals(1, stats.failedSearches());
    assertEquals(1, stats.missedGrabs());
    assertTrue(stats.report().contains("drive stalls=1, drive timeouts=2, turn timeouts=3, "
        + "failed searches=1, missed grabs=1, sonar stops=1"), stats.report());
  }

  @Test
  void anOpenPhaseCountsUpToNowWithoutBeingClosed() {
    MissionStats stats = new MissionStats(() -> now);
    now = 2_000;
    stats.beginPhase("Search");
    now = 5_000;
    assertEquals(3_000L, stats.phases().get("Search"));
    stats.report(); // reporting doesn't close the phase...
    now = 6_000;
    assertEquals(4_000L, stats.phases().get("Search")); // ...so it keeps counting
  }
}
