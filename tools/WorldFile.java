import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Minimal reader for the top-level nodes of a Webots .wbt file, producing 2-D footprints in the
 * controller's cm frame (x = X * 100, y = -Z * 100). Used by the tests (route vs wall clearance)
 * and by {@code MapRenderer}. Not a general VRML parser: it understands the node types in
 * worlds/ProjectWorld2025.wbt and rotations about the vertical (Y) axis.
 */
final class WorldFile {

  enum Kind { WALL, WOODEN_BOX, CARDBOARD_BOX, HONEY_JAR, ROBOT }

  static final class Obstacle {
    final Kind kind;
    final String name;
    final Vec2 center;
    /** Footprint corners in cm, counter-clockwise. Jars get a 10 cm square. */
    final Vec2[] corners;
    final boolean blue;
    final double mass;

    Obstacle(Kind kind, String name, Vec2 center, Vec2[] corners, boolean blue, double mass) {
      this.kind = kind;
      this.name = name;
      this.center = center;
      this.corners = corners;
      this.blue = blue;
      this.mass = mass;
    }

    /** Walls and mass-less wooden boxes cannot be pushed. */
    boolean isStatic() {
      return kind == Kind.WALL || (kind == Kind.WOODEN_BOX && mass <= 0);
    }

    /** Small wooden boxes (20 x 20 cm) mark the drop-off slots. */
    boolean isDropoffPad() {
      return kind == Kind.WOODEN_BOX && corners[0].distanceTo(corners[2]) < 40;
    }
  }

  private static final Pattern NODE_START = Pattern.compile("^(?:DEF \\S+ )?(\\w+) \\{");
  private static final Pattern NUMBERS = Pattern.compile("-?\\d+(?:\\.\\d+)?(?:[eE][-+]?\\d+)?");

  final List<Obstacle> obstacles = new ArrayList<>();
  Vec2 robotStart;

  static WorldFile read(Path path) throws IOException {
    WorldFile world = new WorldFile();
    List<String> lines = Files.readAllLines(path, StandardCharsets.UTF_8);
    int i = 0;
    while (i < lines.size()) {
      Matcher m = NODE_START.matcher(lines.get(i));
      if (!m.find()) {
        i++;
        continue;
      }
      String type = m.group(1);
      List<String> body = new ArrayList<>();
      int depth = 0;
      do {
        String line = lines.get(i);
        body.add(depth + "|" + line.trim()); // remember depth at the start of each line
        depth += count(line, '{') - count(line, '}');
        i++;
      } while (depth > 0 && i < lines.size());
      world.addNode(type, body);
    }
    return world;
  }

  private void addNode(String type, List<String> body) {
    Kind kind;
    switch (type) {
      case "Wall": kind = Kind.WALL; break;
      case "WoodenBox": kind = Kind.WOODEN_BOX; break;
      case "CardboardBox": kind = Kind.CARDBOARD_BOX; break;
      case "HoneyJar": kind = Kind.HONEY_JAR; break;
      case "Pioneer3dx": kind = Kind.ROBOT; break;
      default: return;
    }
    double[] t = field(body, "translation", new double[] {0, 0, 0});
    double[] r = field(body, "rotation", new double[] {0, 1, 0, 0});
    double[] size = field(body, "size", new double[] {0.6, 0.6, 0.6});
    double mass = field(body, "mass", new double[] {0})[0];
    String name = type;
    boolean blue = false;
    for (String entry : body) {
      String line = entry.substring(entry.indexOf('|') + 1);
      if (entry.startsWith("1|name ")) {
        name = line.substring(5).replace("\"", "");
      }
      blue |= line.startsWith("colorOverride 0 0 1");
    }

    Vec2 center = Vec2.of(t[0] * 100, -t[2] * 100);
    if (kind == Kind.ROBOT) {
      robotStart = center;
      return;
    }
    // Only rotations about the vertical axis change the footprint.
    double yaw = Math.abs(r[1]) > 0.9 ? r[3] * Math.signum(r[1]) : 0;
    double hx = (kind == Kind.HONEY_JAR ? 0.10 : size[0]) / 2;
    double sz = kind == Kind.HONEY_JAR ? 0.10 : size[2];
    // Webots' Wall proto offsets its box by +size.z/2 along local z; boxes are centred.
    double z0 = kind == Kind.WALL ? 0 : -sz / 2;
    double z1 = kind == Kind.WALL ? sz : sz / 2;
    double[][] local = {{-hx, z0}, {hx, z0}, {hx, z1}, {-hx, z1}};
    Vec2[] corners = new Vec2[4];
    double c = Math.cos(yaw);
    double s = Math.sin(yaw);
    for (int k = 0; k < 4; k++) {
      double lx = local[k][0];
      double lz = local[k][1];
      double worldX = t[0] + lx * c + lz * s;
      double worldZ = t[2] - lx * s + lz * c;
      corners[k] = Vec2.of(worldX * 100, -worldZ * 100);
    }
    obstacles.add(new Obstacle(kind, name, center, corners, blue, mass));
  }

  List<Obstacle> ofKind(Kind kind) {
    List<Obstacle> result = new ArrayList<>();
    for (Obstacle o : obstacles) {
      if (o.kind == kind) {
        result.add(o);
      }
    }
    return result;
  }

  List<Obstacle> staticObstacles() {
    List<Obstacle> result = new ArrayList<>();
    for (Obstacle o : obstacles) {
      if (o.isStatic()) {
        result.add(o);
      }
    }
    return result;
  }

  // ------------------------------------------------------------------ geometry helpers

  /** Shortest distance (cm) between segment a-b and the obstacle footprint; 0 if they touch. */
  static double clearance(Vec2 a, Vec2 b, Obstacle o) {
    if (contains(o.corners, a) || contains(o.corners, b)) {
      return 0;
    }
    double best = Double.MAX_VALUE;
    for (int k = 0; k < 4; k++) {
      Vec2 p = o.corners[k];
      Vec2 q = o.corners[(k + 1) % 4];
      best = Math.min(best, segmentDistance(a, b, p, q));
    }
    return best;
  }

  static boolean contains(Vec2[] polygon, Vec2 p) {
    boolean inside = false;
    for (int i = 0, j = polygon.length - 1; i < polygon.length; j = i++) {
      Vec2 pi = polygon[i];
      Vec2 pj = polygon[j];
      if ((pi.y > p.y) != (pj.y > p.y)
          && p.x < (pj.x - pi.x) * (p.y - pi.y) / (pj.y - pi.y) + pi.x) {
        inside = !inside;
      }
    }
    return inside;
  }

  static double segmentDistance(Vec2 a, Vec2 b, Vec2 c, Vec2 d) {
    if (segmentsIntersect(a, b, c, d)) {
      return 0;
    }
    return Math.min(Math.min(pointToSegment(a, c, d), pointToSegment(b, c, d)),
        Math.min(pointToSegment(c, a, b), pointToSegment(d, a, b)));
  }

  private static boolean segmentsIntersect(Vec2 a, Vec2 b, Vec2 c, Vec2 d) {
    double d1 = cross(c, d, a);
    double d2 = cross(c, d, b);
    double d3 = cross(a, b, c);
    double d4 = cross(a, b, d);
    return ((d1 > 0) != (d2 > 0)) && ((d3 > 0) != (d4 > 0));
  }

  private static double cross(Vec2 o, Vec2 a, Vec2 b) {
    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
  }

  static double pointToSegment(Vec2 p, Vec2 a, Vec2 b) {
    double dx = b.x - a.x;
    double dy = b.y - a.y;
    double len2 = dx * dx + dy * dy;
    double t = len2 == 0 ? 0 : ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2;
    t = Math.max(0, Math.min(1, t));
    return p.distanceTo(Vec2.of(a.x + t * dx, a.y + t * dy));
  }

  private static double[] field(List<String> body, String fieldName, double[] fallback) {
    String prefix = "1|" + fieldName + " ";
    for (String entry : body) {
      if (entry.startsWith(prefix)) {
        Matcher m = NUMBERS.matcher(entry.substring(prefix.length()));
        List<Double> values = new ArrayList<>();
        while (m.find()) {
          values.add(Double.parseDouble(m.group()));
        }
        double[] out = new double[values.size()];
        for (int k = 0; k < out.length; k++) {
          out[k] = values.get(k);
        }
        return out.length >= fallback.length ? out : fallback;
      }
    }
    return fallback;
  }

  private static int count(String s, char ch) {
    int n = 0;
    for (int k = 0; k < s.length(); k++) {
      if (s.charAt(k) == ch) {
        n++;
      }
    }
    return n;
  }

  @Override
  public String toString() {
    return String.format(Locale.ROOT, "WorldFile[%d obstacles, robot at %s]", obstacles.size(), robotStart);
  }
}
