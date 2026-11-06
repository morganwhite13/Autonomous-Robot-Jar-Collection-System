import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Random;

/**
 * Writes a variant of a world file for robustness runs: the five jars are placed at random in
 * the jar bay, and the robot's start pose is nudged. Seed 0 leaves the world unchanged, so it
 * reproduces the standard run.
 *
 * <p>Usage: {@code java WorldShuffle world.wbt seed} rewrites the file in place. Only the
 * robot's translation and rotation and the jars' translations change; everything else is
 * copied line for line.
 */
public class WorldShuffle {

  /** Where jars may go: the jar bay, within the distances the original jars span. */
  static final double JAR_MIN_X = 820;
  static final double JAR_MAX_X = 950;
  static final double JAR_MIN_Y = -115;
  static final double JAR_MAX_Y = 15;
  /** Centre-to-centre spacing between jars (each is 9 cm across). */
  static final double JAR_SPACING_CM = 25;
  /** Gap between a jar's centre and any wall or box. */
  static final double JAR_CLEARANCE_CM = 12;
  /** Height the jars are dropped from (the original world drops them from 12-21 cm). */
  static final double JAR_DROP_Y = 0.15;

  /** Start pose nudge: up to this far from the original position, and this much heading. */
  static final double START_SHIFT_CM = 8;
  static final double START_TURN_DEG = 6;
  /** The robot's footprint radius for the start-position check (the Pioneer is 45 x 38 cm). */
  static final double ROBOT_CLEARANCE_CM = 26;

  /** The variant for {@code seed}. */
  static final class Variant {
    final List<Vec2> jars;
    final Vec2 start;
    /** Rotation about the vertical axis from the original pose, degrees (left is positive). */
    final double startTurnDeg;

    Variant(List<Vec2> jars, Vec2 start, double startTurnDeg) {
      this.jars = jars;
      this.start = start;
      this.startTurnDeg = startTurnDeg;
    }
  }

  public static void main(String[] args) throws IOException {
    Path world = Path.of(args[0]);
    long seed = Long.parseLong(args[1]);
    if (seed == 0) {
      return;
    }
    WorldFile parsed = WorldFile.read(world);
    Variant v = variant(parsed, seed);
    List<String> lines = Files.readAllLines(world, StandardCharsets.UTF_8);
    Files.write(world, apply(lines, v), StandardCharsets.UTF_8);
    StringBuilder msg = new StringBuilder(String.format(Locale.ROOT,
        "seed %d: start %s turned %+.1f deg; jars", seed, v.start, v.startTurnDeg));
    for (Vec2 jar : v.jars) {
      msg.append(' ').append(jar);
    }
    System.out.println(msg);
  }

  static Variant variant(WorldFile world, long seed) {
    Random random = new Random(seed);
    List<Vec2[]> solids = new ArrayList<>();
    for (WorldFile.Obstacle o : world.obstacles) {
      if (o.kind != WorldFile.Kind.HONEY_JAR) {
        solids.add(o.corners);
      }
    }
    // Never start closer to a box than the original start is.
    double needed = Math.min(ROBOT_CLEARANCE_CM, clearance(world.robotStart, solids));
    Vec2 start;
    do {
      double angle = random.nextDouble() * 2 * Math.PI;
      double r = START_SHIFT_CM * Math.sqrt(random.nextDouble());
      start = Vec2.of(world.robotStart.x + r * Math.cos(angle),
          world.robotStart.y + r * Math.sin(angle));
    } while (clearance(start, solids) < needed);
    double turn = (random.nextDouble() * 2 - 1) * START_TURN_DEG;

    int count = world.ofKind(WorldFile.Kind.HONEY_JAR).size();
    List<Vec2> jars = new ArrayList<>();
    while (jars.size() < count) {
      Vec2 p = Vec2.of(JAR_MIN_X + random.nextDouble() * (JAR_MAX_X - JAR_MIN_X),
          JAR_MIN_Y + random.nextDouble() * (JAR_MAX_Y - JAR_MIN_Y));
      if (clearance(p, solids) < JAR_CLEARANCE_CM) {
        continue;
      }
      boolean spaced = true;
      for (Vec2 other : jars) {
        spaced &= p.distanceTo(other) >= JAR_SPACING_CM;
      }
      if (spaced) {
        jars.add(p);
      }
    }
    return new Variant(jars, start, turn);
  }

  private static double clearance(Vec2 p, List<Vec2[]> solids) {
    double nearest = Double.POSITIVE_INFINITY;
    for (Vec2[] polygon : solids) {
      nearest = Math.min(nearest, RoutePlanner.distanceToPolygon(p, polygon));
    }
    return nearest;
  }

  /** The world's lines with the robot's pose and the jars' positions replaced. */
  static List<String> apply(List<String> lines, Variant v) {
    List<String> out = new ArrayList<>();
    String node = "";
    int depth = 0;
    int jar = 0;
    for (String line : lines) {
      String trimmed = line.trim();
      if (depth == 0 && trimmed.endsWith("{")) {
        String[] words = trimmed.split(" ");
        node = words[0].equals("DEF") ? words[2] : words[0];
      }
      String indent = line.substring(0, line.length() - line.stripLeading().length());
      if (depth == 1 && node.equals("Pioneer3dx") && trimmed.startsWith("translation ")) {
        double y = Double.parseDouble(trimmed.split(" ")[2]);
        line = indent + String.format(Locale.ROOT, "translation %.5f %s %.5f",
            v.start.x / 100, fmt(y), -v.start.y / 100);
      } else if (depth == 1 && node.equals("Pioneer3dx") && trimmed.startsWith("rotation ")) {
        line = indent + String.format(Locale.ROOT, "rotation 0 1 0 %.6f",
            Math.toRadians(v.startTurnDeg));
      } else if (depth == 1 && node.equals("HoneyJar") && trimmed.startsWith("translation ")) {
        Vec2 p = v.jars.get(jar++);
        line = indent + String.format(Locale.ROOT, "translation %.5f %s %.5f",
            p.x / 100, fmt(JAR_DROP_Y), -p.y / 100);
      }
      out.add(line);
      depth += count(line, '{') - count(line, '}');
    }
    return out;
  }

  private static String fmt(double d) {
    return String.format(Locale.ROOT, "%.6f", d);
  }

  private static int count(String s, char c) {
    int n = 0;
    for (int i = 0; i < s.length(); i++) {
      if (s.charAt(i) == c) {
        n++;
      }
    }
    return n;
  }
}
