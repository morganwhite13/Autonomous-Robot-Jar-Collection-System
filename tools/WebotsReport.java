import java.io.IOException;
import java.io.StringReader;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;
import java.util.Properties;

/**
 * Turns the output of tools/run-in-webots.sh (one folder per run, each holding result.properties,
 * trajectory.csv and webots.log) into a Markdown comparison.
 *
 * <p>Usage: {@code java WebotsReport build/webots-run v1 v2}
 */
public class WebotsReport {

  /** Log messages that mean "something went wrong and was recovered from", per controller. */
  static final String[][] EVENTS = {
      {"Approach aborted",
          "Approach taking too long", "Lost green for too long", "Approach timeout reached",
          "Lost sight of the jar", "Not making progress", "Approach timed out"},
      {"Full scan saw no jar", "No green detected in full rotation", "No jar seen in a full turn"},
      {"Whole search failed", "CRITICAL: Failed to find jar", "search failed ("},
      {"Drive stalled or timed out (v2 detects these; v1 cannot)", "WARNING: stalled at",
          "WARNING: drive to", "WARNING: turn to"},
  };

  /** A run that ends this close to where it has been for the last two minutes counts as stuck. */
  static final double STUCK_RADIUS_CM = 10;
  static final double STUCK_MIN_S = 120;

  static final class Run {
    final String name;
    final Properties result;
    final List<Vec2> path;
    /** Simulated time of each point in {@link #path}, in seconds. */
    final List<Double> times;
    final List<String> log;

    Run(String name, Properties result, List<Vec2> path, List<Double> times, List<String> log) {
      this.name = name;
      this.result = result;
      this.path = path;
      this.times = times;
      this.log = log;
    }

    /**
     * Jars the controller itself says it delivered: both print "Jars remaining: N" once per jar
     * they think they placed. v1 never checks that it is holding anything.
     */
    long claimed() {
      return count("Jars remaining: ");
    }

    /**
     * For a run that was stopped at the time limit: when the robot came to rest within
     * {@link #STUCK_RADIUS_CM} of where it ended, if that was at least {@link #STUCK_MIN_S} before
     * the end. NaN otherwise.
     */
    double stuckFromS() {
      if (!Boolean.parseBoolean(result.getProperty("timeLimitHit", "false")) || path.isEmpty()) {
        return Double.NaN;
      }
      Vec2 end = path.get(path.size() - 1);
      int i = path.size() - 1;
      while (i > 0 && path.get(i - 1).distanceTo(end) <= STUCK_RADIUS_CM) {
        i--;
      }
      double from = times.get(i);
      return times.get(times.size() - 1) - from >= STUCK_MIN_S ? from : Double.NaN;
    }

    int delivered() {
      return Integer.parseInt(result.getProperty("delivered", "0"));
    }

    int jars() {
      return Integer.parseInt(result.getProperty("jars", "0"));
    }

    double simTime() {
      return Double.parseDouble(result.getProperty("simTimeS", "0"));
    }

    List<Double> deliveryTimes() {
      List<Double> times = new ArrayList<>();
      for (String t : result.getProperty("deliveryTimesS", "").split(",")) {
        if (!t.isEmpty()) {
          times.add(Double.parseDouble(t));
        }
      }
      return times;
    }

    long count(String... phrases) {
      return log.stream().filter(line -> {
        for (String p : phrases) {
          if (line.contains(p)) {
            return true;
          }
        }
        return false;
      }).count();
    }
  }

  public static void main(String[] args) throws IOException {
    Path root = Path.of(args[0]);
    List<Run> runs = new ArrayList<>();
    for (int i = 1; i < args.length; i++) {
      runs.add(load(root.resolve(args[i]), args[i]));
    }
    System.out.print(markdown(runs));
  }

  static Run load(Path dir, String name) throws IOException {
    Properties result = new Properties();
    Path resultFile = dir.resolve("result.properties");
    if (Files.exists(resultFile)) {
      result.load(new StringReader(Files.readString(resultFile, StandardCharsets.UTF_8)));
    }
    Path csv = dir.resolve("trajectory.csv");
    List<Vec2> path = new ArrayList<>();
    List<Double> times = new ArrayList<>();
    if (Files.exists(csv)) {
      for (String line : Files.readAllLines(csv, StandardCharsets.UTF_8)) {
        String[] f = line.split(",");
        if (f.length == 3 && !f[0].equals("t")) {
          times.add(Double.parseDouble(f[0]));
          path.add(Vec2.of(Double.parseDouble(f[1]), Double.parseDouble(f[2])));
        }
      }
    }
    Path logFile = dir.resolve("webots.log");
    List<String> log = Files.exists(logFile)
        ? Files.readAllLines(logFile, StandardCharsets.ISO_8859_1)
        : Collections.emptyList();
    return new Run(name, result, path, times, log);
  }

  static double distanceM(List<Vec2> path) {
    double cm = 0;
    for (int i = 1; i < path.size(); i++) {
      cm += path.get(i - 1).distanceTo(path.get(i));
    }
    return cm / 100;
  }

  static String markdown(List<Run> runs) {
    StringBuilder md = new StringBuilder();
    md.append("| |");
    for (Run r : runs) {
      md.append(' ').append(r.name).append(" |");
    }
    md.append("\n|---|");
    for (int i = 0; i < runs.size(); i++) {
      md.append("---|");
    }
    md.append('\n');

    row(md, runs, "Jars delivered", r -> r.delivered() + "/" + r.jars());
    row(md, runs, "Jars the controller says it delivered", r -> Long.toString(r.claimed()));
    row(md, runs, "Mission time (last jar delivered)", r -> r.delivered() == r.jars() && r.jars() > 0
        ? minutes(r.deliveryTimes().get(r.deliveryTimes().size() - 1))
        : "did not finish (stopped at " + minutes(r.simTime()) + ")");
    row(md, runs, "Jar delivery times (s)", r -> {
      StringBuilder s = new StringBuilder();
      for (double t : r.deliveryTimes()) {
        s.append(s.length() == 0 ? "" : ", ").append(String.format(Locale.ROOT, "%.0f", t));
      }
      return s.length() == 0 ? "-" : s.toString();
    });
    row(md, runs, "Stuck in one place from", r -> Double.isNaN(r.stuckFromS())
        ? "-" : String.format(Locale.ROOT, "%.0f s to the end", r.stuckFromS()));
    row(md, runs, "Distance driven", r -> String.format(Locale.ROOT, "%.0f m", distanceM(r.path)));
    row(md, runs, "Jars knocked out of the drop-off lane afterwards",
        r -> r.result.getProperty("disturbances", "-"));
    for (String[] event : EVENTS) {
      String[] phrases = new String[event.length - 1];
      System.arraycopy(event, 1, phrases, 0, phrases.length);
      row(md, runs, event[0], r -> Long.toString(r.count(phrases)));
    }
    return md.toString();
  }

  private interface Cell {
    String of(Run r);
  }

  private static void row(StringBuilder md, List<Run> runs, String label, Cell cell) {
    md.append("| ").append(label).append(" |");
    for (Run r : runs) {
      md.append(' ').append(cell.of(r)).append(" |");
    }
    md.append('\n');
  }

  private static String minutes(double seconds) {
    return String.format(Locale.ROOT, "%.0f s (%.1f min)", seconds, seconds / 60);
  }
}
