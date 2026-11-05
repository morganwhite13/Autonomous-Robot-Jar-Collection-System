import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * Draws the arena from the .wbt file as an SVG, either with the planned waypoint routes from
 * {@link Routes} (the README map) or with paths actually driven in Webots (recorded by
 * tools/webots/Observer.java), so the pictures come from the same data the controllers use.
 *
 * <p>Usage:
 * <pre>
 *   java MapRenderer worlds/ProjectWorld2025.wbt docs/images/mission-map.svg
 *   java MapRenderer worlds/ProjectWorld2025.wbt out.svg --path v1=run/v1/trajectory.csv --path v2=...
 * </pre>
 */
public class MapRenderer {

  // Visible window in controller cm. The southern part of the arena (y < -260) is unused.
  private static final double MIN_X = -1010;
  private static final double MAX_X = 1010;
  private static final double MIN_Y = -260;
  private static final double MAX_Y = 1010;
  private static final double LEGEND_H = 230;
  // Reference palette categorical slots 2, 1, 3, 4: v1 orange, v2 blue, as in before-after.svg.
  private static final String[] PATH_COLORS = {"#eb6834", "#2a78d6", "#1baf7a", "#eda100"};

  private final StringBuilder svg = new StringBuilder();
  /** Obstacles left out of the drawing (e.g. boxes assumed pushed aside). */
  private Collection<WorldFile.Obstacle> hidden = Collections.emptyList();

  public static void main(String[] args) throws IOException {
    WorldFile world = WorldFile.read(Path.of(args[0]));
    Map<String, List<Vec2>> paths = new LinkedHashMap<>();
    for (int i = 2; i + 1 < args.length; i += 2) {
      if (!args[i].equals("--path") || !args[i + 1].contains("=")) {
        throw new IllegalArgumentException("expected --path name=trajectory.csv, got " + args[i]);
      }
      String[] spec = args[i + 1].split("=", 2);
      paths.put(spec[0], readTrajectory(Path.of(spec[1])));
    }
    String out = paths.isEmpty()
        ? new MapRenderer().render(world)
        : new MapRenderer().renderPaths(world, paths);
    Files.write(Path.of(args[1]), out.getBytes(StandardCharsets.UTF_8));
  }

  /** Reads "t,x,y" rows (cm) written by the observer. */
  static List<Vec2> readTrajectory(Path csv) throws IOException {
    List<Vec2> points = new ArrayList<>();
    for (String line : Files.readAllLines(csv, StandardCharsets.UTF_8)) {
      String[] f = line.split(",");
      if (f.length == 3 && !f[0].equals("t")) {
        points.add(Vec2.of(Double.parseDouble(f[1]), Double.parseDouble(f[2])));
      }
    }
    return points;
  }

  /** Paths driven in Webots, one colour per run, over the arena. */
  String renderPaths(WorldFile world, Map<String, List<Vec2>> paths) {
    return renderPaths(world, paths, Collections.emptyList(), "Paths driven in Webots",
        "Cardboard box: pushable (start positions)");
  }

  /** Paths over the arena, leaving out {@code hiddenBoxes}, with the given title and box legend. */
  String renderPaths(WorldFile world, Map<String, List<Vec2>> paths,
      Collection<WorldFile.Obstacle> hiddenBoxes, String title, String cardboardLegend) {
    hidden = hiddenBoxes;
    open(title, new String[0][]);
    drawObstacles(world);
    int i = 0;
    for (List<Vec2> path : paths.values()) {
      StringBuilder pts = new StringBuilder();
      for (Vec2 p : path) {
        pts.append(String.format(Locale.ROOT, "%.1f,%.1f ", sx(p.x), sy(p.y)));
      }
      add("<polyline points=\"%s\" fill=\"none\" stroke=\"%s\" stroke-width=\"6\" "
          + "stroke-opacity=\"0.75\" stroke-linejoin=\"round\" stroke-linecap=\"round\"/>",
          pts.toString().trim(), PATH_COLORS[i++ % PATH_COLORS.length]);
    }
    drawMarkers(world);
    add("</g>");
    double y = MAX_Y - MIN_Y + 60;
    i = 0;
    for (Map.Entry<String, List<Vec2>> e : paths.entrySet()) {
      add("<line x1=\"30\" y1=\"%.0f\" x2=\"120\" y2=\"%.0f\" stroke=\"%s\" stroke-width=\"8\"/>",
          y, y, PATH_COLORS[i++ % PATH_COLORS.length]);
      add("<text x=\"140\" y=\"%.0f\" font-size=\"34\" fill=\"#212529\">%s</text>", y + 12, e.getKey());
      y += 48;
    }
    obstacleLegend(1060, MAX_Y - MIN_Y + 60, cardboardLegend);
    add("</svg>");
    return svg.toString();
  }

  String render(WorldFile world) {
    open("Jar collection mission map", new String[][] {{"clear", "#d9480f"}, {"carry", "#1c7ed6"},
        {"back", "#7048e8"}, {"bug", "#e03131"}});
    drawObstacles(world);

    List<Vec2> clearing = new ArrayList<>();
    clearing.add(world.robotStart);
    clearing.addAll(Routes.CLEARING_PATH);
    route(clearing, "#d9480f", "clear", "none", 7);
    List<Vec2> carry = new ArrayList<>();
    carry.add(Routes.SEARCH_POINT);
    carry.addAll(Routes.SEARCH_TO_DROPOFF);
    route(carry, "#1c7ed6", "carry", "none", 7);
    route(Routes.DROPOFF_TO_SEARCH, "#7048e8", "back", "18 12", 5);
    route(List.of(Routes.SEARCH_POINT, Routes.DROPOFF_ENTRY), "#e03131", "bug", "6 14", 5);

    drawMarkers(world);
    label(-60, 470, "v1 retry after a failed search (jars 2-5): ", "#e03131", "middle");
    label(-60, 430, "straight through 4 static boxes. Fixed in v2.", "#e03131", "middle");
    add("</g>");

    legend(MAX_Y - MIN_Y + 20);
    add("</svg>");
    return svg.toString();
  }

  /** SVG header, arrow markers, background, and the start of the clipped arena group. */
  private void open(String title, String[][] markers) {
    double w = MAX_X - MIN_X;
    double h = MAX_Y - MIN_Y + LEGEND_H;
    add("<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 %.0f %.0f\" width=\"1000\" "
        + "font-family=\"Helvetica, Arial, sans-serif\">", w, h);
    add("<title>%s</title>", title);
    add("<defs>");
    for (String[] m : markers) {
      add("<marker id=\"arrow-%s\" viewBox=\"0 0 10 10\" refX=\"8\" refY=\"5\" markerWidth=\"5\" "
          + "markerHeight=\"5\" orient=\"auto-start-reverse\"><path d=\"M0,0 L10,5 L0,10 z\" "
          + "fill=\"%s\"/></marker>", m[0], m[1]);
    }
    add("<clipPath id=\"arena\"><rect x=\"0\" y=\"0\" width=\"%.0f\" height=\"%.0f\"/></clipPath>",
        w, MAX_Y - MIN_Y);
    add("</defs>");
    add("<rect width=\"100%%\" height=\"100%%\" fill=\"#fbf8f2\"/>");
    add("<g clip-path=\"url(#arena)\">");
    add("<rect x=\"0\" y=\"0\" width=\"%.0f\" height=\"%.0f\" fill=\"#f1e6d2\"/>", w, MAX_Y - MIN_Y);
  }

  private void drawObstacles(WorldFile world) {
    for (WorldFile.Obstacle o : world.ofKind(WorldFile.Kind.CARDBOARD_BOX)) {
      if (hidden.contains(o)) {
        continue;
      }
      polygon(o.corners, "#e2c391", "#a07a3c", 2);
    }
    for (WorldFile.Obstacle o : world.ofKind(WorldFile.Kind.WOODEN_BOX)) {
      polygon(o.corners, o.isDropoffPad() ? "#5c3d1e" : "#8a5a2b", "#3d2810", 2);
    }
    for (WorldFile.Obstacle o : world.ofKind(WorldFile.Kind.WALL)) {
      polygon(o.corners, o.blue ? "#1971c2" : "#495057", "none", 0);
    }
  }

  /** Jars, start position, search point and place labels. */
  private void drawMarkers(WorldFile world) {
    for (WorldFile.Obstacle o : world.ofKind(WorldFile.Kind.HONEY_JAR)) {
      circle(o.center, 9, "#2f9e44", "#1b5e20");
    }
    circle(world.robotStart, 22, "#212529", "#ffffff");
    star(Routes.SEARCH_POINT);

    label(world.robotStart.x + 30, world.robotStart.y - 45, "start", "#212529", "start");
    label(Routes.SEARCH_POINT.x - 45, Routes.SEARCH_POINT.y - 45, "search point", "#212529", "end");
    label(905, -150, "jar bay", "#2b8a3e", "middle");
    label(-930, 760, "drop-off", "#5c3d1e", "start");
  }

  private void legend(double top) {
    double x = 30;
    double y = top + 40;
    String[][] lines = {
        {"#d9480f", "none", "Phase 1: box-clearing path (v1 = v2)"},
        {"#1c7ed6", "none", "Carry jar to drop-off (v1 = v2)"},
        {"#7048e8", "18 12", "Return from drop-off (v1 = v2)"},
        {"#e03131", "6 14", "v1 only: route taken after a failed search"},
    };
    for (String[] l : lines) {
      add("<line x1=\"%.0f\" y1=\"%.0f\" x2=\"%.0f\" y2=\"%.0f\" stroke=\"%s\" stroke-width=\"8\" "
          + "stroke-dasharray=\"%s\"/>", x, y, x + 90, y, l[0], l[1]);
      add("<text x=\"%.0f\" y=\"%.0f\" font-size=\"34\" fill=\"#212529\">%s</text>", x + 110, y + 12, l[2]);
      y += 48;
    }
    obstacleLegend(1060, top + 40, "Cardboard box: pushable (start positions)");
  }

  private void obstacleLegend(double x2, double y2, String cardboardText) {
    swatch(x2, y2, "#495057", "Wall (blue = coloured panel)");
    swatch(x2, y2 + 48, "#8a5a2b", "Wooden box: static, cannot be pushed");
    swatch(x2, y2 + 96, "#e2c391", cardboardText);
    add("<circle cx=\"%.0f\" cy=\"%.0f\" r=\"11\" fill=\"#2f9e44\"/>", x2 + 20, y2 + 144);
    add("<text x=\"%.0f\" y=\"%.0f\" font-size=\"34\" fill=\"#212529\">Honey jar (5)</text>", x2 + 50, y2 + 156);
  }

  private void swatch(double x, double y, String fill, String text) {
    add("<rect x=\"%.0f\" y=\"%.0f\" width=\"40\" height=\"26\" fill=\"%s\"/>", x, y - 13, fill);
    add("<text x=\"%.0f\" y=\"%.0f\" font-size=\"34\" fill=\"#212529\">%s</text>", x + 50, y + 12, text);
  }

  private void polygon(Vec2[] corners, String fill, String stroke, double strokeWidth) {
    StringBuilder pts = new StringBuilder();
    for (Vec2 c : corners) {
      pts.append(String.format(Locale.ROOT, "%.1f,%.1f ", sx(c.x), sy(c.y)));
    }
    add("<polygon points=\"%s\" fill=\"%s\" stroke=\"%s\" stroke-width=\"%.0f\"/>",
        pts.toString().trim(), fill, stroke, strokeWidth);
  }

  private void route(List<Vec2> pts, String color, String marker, String dash, double width) {
    for (int i = 0; i + 1 < pts.size(); i++) {
      Vec2 a = pts.get(i);
      Vec2 b = pts.get(i + 1);
      add("<line x1=\"%.1f\" y1=\"%.1f\" x2=\"%.1f\" y2=\"%.1f\" stroke=\"%s\" stroke-width=\"%.0f\" "
          + "stroke-dasharray=\"%s\" stroke-linecap=\"round\" marker-end=\"url(#arrow-%s)\"/>",
          sx(a.x), sy(a.y), sx(b.x), sy(b.y), color, width, dash, marker);
    }
  }

  private void circle(Vec2 c, double r, String fill, String stroke) {
    add("<circle cx=\"%.1f\" cy=\"%.1f\" r=\"%.0f\" fill=\"%s\" stroke=\"%s\" stroke-width=\"3\"/>",
        sx(c.x), sy(c.y), r, fill, stroke);
  }

  private void star(Vec2 c) {
    StringBuilder pts = new StringBuilder();
    for (int k = 0; k < 10; k++) {
      double r = k % 2 == 0 ? 30 : 12;
      double a = Math.toRadians(-90 + k * 36);
      pts.append(String.format(Locale.ROOT, "%.1f,%.1f ", sx(c.x) + r * Math.cos(a), sy(c.y) + r * Math.sin(a)));
    }
    add("<polygon points=\"%s\" fill=\"#fcc419\" stroke=\"#212529\" stroke-width=\"3\"/>", pts.toString().trim());
  }

  private void label(double x, double y, String text, String color, String anchor) {
    add("<text x=\"%.1f\" y=\"%.1f\" font-size=\"36\" font-weight=\"bold\" fill=\"%s\" "
        + "text-anchor=\"%s\" stroke=\"#fbf8f2\" stroke-width=\"8\" paint-order=\"stroke\">%s</text>",
        sx(x), sy(y), color, anchor, text);
  }

  private static double sx(double x) {
    return x - MIN_X;
  }

  private static double sy(double y) {
    return MAX_Y - y;
  }

  private void add(String format, Object... args) {
    svg.append(String.format(Locale.ROOT, format, args)).append('\n');
  }
}
