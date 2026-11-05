import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.function.LongSupplier;

/**
 * Mission telemetry: time per phase, per-jar effort, and anomalies. v1 only printed progress
 * messages, so there was no way to compare runs; v2 ends every run with {@link #report()}.
 */
final class MissionStats {

  /** Effort spent on one jar. A jar may need several search attempts. */
  static final class JarRecord {
    final int number;
    final long startMs;
    long endMs = -1;
    int scans;
    int approaches;
    boolean delivered;

    JarRecord(int number, long startMs) {
      this.number = number;
      this.startMs = startMs;
    }
  }

  private final LongSupplier clockMs;
  private final Map<String, Long> phaseMs = new LinkedHashMap<>();
  private final List<JarRecord> jars = new ArrayList<>();
  private String currentPhase;
  private long phaseStartMs;
  private JarRecord currentJar;

  private int driveStalls;
  private int driveTimeouts;
  private int turnTimeouts;
  private int failedSearches;
  private int missedGrabs;

  MissionStats(LongSupplier clockMs) {
    this.clockMs = clockMs;
  }

  /** Starts timing {@code name}; time is accumulated if the same phase repeats (once per jar). */
  void beginPhase(String name) {
    endPhase();
    currentPhase = name;
    phaseStartMs = clockMs.getAsLong();
  }

  void endPhase() {
    if (currentPhase != null) {
      phaseMs.merge(currentPhase, clockMs.getAsLong() - phaseStartMs, Long::sum);
      currentPhase = null;
    }
  }

  /** Starts a record for jar {@code number}, or keeps the open one if it is the same jar. */
  void beginJar(int number) {
    if (currentJar != null && currentJar.number == number && currentJar.endMs < 0) {
      return;
    }
    currentJar = new JarRecord(number, clockMs.getAsLong());
    jars.add(currentJar);
  }

  void recordScan() {
    if (currentJar != null) {
      currentJar.scans++;
    }
  }

  void recordApproach() {
    if (currentJar != null) {
      currentJar.approaches++;
    }
  }

  void recordFailedSearch() {
    failedSearches++;
  }

  void recordMissedGrab() {
    missedGrabs++;
  }

  void recordDriveStall() {
    driveStalls++;
  }

  void recordDriveTimeout() {
    driveTimeouts++;
  }

  void recordTurnTimeout() {
    turnTimeouts++;
  }

  int driveStalls() {
    return driveStalls;
  }

  int driveTimeouts() {
    return driveTimeouts;
  }

  int turnTimeouts() {
    return turnTimeouts;
  }

  int failedSearches() {
    return failedSearches;
  }

  /** Grabs the grab check undid because the fingers weren't holding anything. */
  int missedGrabs() {
    return missedGrabs;
  }

  /** Drive stalls, drive timeouts and turn timeouts: legs that didn't end where planned. */
  int navigationProblems() {
    return driveStalls + driveTimeouts + turnTimeouts;
  }

  void jarDelivered() {
    if (currentJar != null) {
      currentJar.delivered = true;
      currentJar.endMs = clockMs.getAsLong();
    }
  }

  int jarsDelivered() {
    int n = 0;
    for (JarRecord j : jars) {
      if (j.delivered) {
        n++;
      }
    }
    return n;
  }

  List<JarRecord> jars() {
    return Collections.unmodifiableList(jars);
  }

  /** Time per phase so far, in the order the phases first ran (an open phase counts up to now). */
  Map<String, Long> phases() {
    Map<String, Long> totals = new LinkedHashMap<>(phaseMs);
    if (currentPhase != null) {
      totals.merge(currentPhase, clockMs.getAsLong() - phaseStartMs, Long::sum);
    }
    return Collections.unmodifiableMap(totals);
  }

  String report() {
    long now = clockMs.getAsLong();
    StringBuilder sb = new StringBuilder();
    sb.append("==================== MISSION REPORT (v2) ====================\n");
    sb.append(String.format(Locale.ROOT,
        "Result: %d/%d jars delivered in %.1f s of simulated time%n",
        jarsDelivered(), Config.JAR_COUNT, now / 1000.0));
    sb.append("\nTime by phase (summed over all jars)\n");
    for (Map.Entry<String, Long> e : phases().entrySet()) {
      sb.append(String.format(Locale.ROOT, "  %-24s %8.1f s%n", e.getKey(), e.getValue() / 1000.0));
    }
    sb.append("\nJar   Time (s)  Scans  Approaches  Result\n");
    for (JarRecord j : jars) {
      long end = j.endMs >= 0 ? j.endMs : now;
      sb.append(String.format(Locale.ROOT, "  %-3d %8.1f  %5d  %10d  %s%n",
          j.number, (end - j.startMs) / 1000.0, j.scans, j.approaches,
          j.delivered ? "delivered" : "not delivered"));
    }
    sb.append(String.format(Locale.ROOT,
        "%nAnomalies: drive stalls=%d, drive timeouts=%d, turn timeouts=%d, failed searches=%d, "
            + "missed grabs=%d%n",
        driveStalls, driveTimeouts, turnTimeouts, failedSearches, missedGrabs));
    sb.append("==============================================================");
    return sb.toString();
  }
}
