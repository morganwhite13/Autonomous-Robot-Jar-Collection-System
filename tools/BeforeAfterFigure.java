import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Random;

/**
 * Draws docs/images/before-after.svg: three small panels comparing v1 and v2, each computed on
 * the kinematic model the same way as {@link BeforeAfterBenchmark} (v1 via the ports in
 * tests/LegacyV1.java, v2 via the real classes).
 *
 * <p>Usage: {@code java BeforeAfterFigure docs/images/before-after.svg}
 */
public class BeforeAfterFigure {

  // Reference palette (dataviz skill): categorical slots 2 and 1, ink and surface tokens.
  static final String V1 = "#eb6834";
  static final String V2 = "#2a78d6";
  static final String INK = "#0b0b0b";
  static final String INK_2 = "#52514e";
  static final String GRID = "#e4e2dd";
  static final String SURFACE = "#fcfcfb";

  static final double W = 1200;
  static final double H = 395;
  static final double PANEL_W = 360;
  static final double GAP = 45;
  static final double LEFT = 30;
  static final double TOP = 150;
  static final double PLOT_H = 200;

  private final StringBuilder svg = new StringBuilder();

  public static void main(String[] args) throws IOException {
    Log.useSink(line -> { });
    Files.write(Path.of(args[0]), new BeforeAfterFigure().render().getBytes(StandardCharsets.UTF_8));
  }

  String render() {
    add("<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 %.0f %.0f\" width=\"1000\" "
        + "font-family=\"Helvetica, Arial, sans-serif\">", W, H);
    add("<title>v1 vs v2 on the same robot model</title>");
    add("<rect width=\"100%%\" height=\"100%%\" fill=\"%s\"/>", SURFACE);
    add("<text x=\"%.0f\" y=\"44\" font-size=\"26\" font-weight=\"bold\" fill=\"%s\">"
        + "v1 vs v2, measured on the same robot model</text>", LEFT, INK);
    legendSwatch(LEFT, 78, V1, "v1 (original)");
    legendSwatch(LEFT + 190, 78, V2, "v2 (improved)");

    scanPanel(LEFT);
    driftPanel(LEFT + PANEL_W + GAP);
    retryPanel(LEFT + 2 * (PANEL_W + GAP));
    add("</svg>");
    return svg.toString();
  }

  // ------------------------------------------------------------------ panel 1: scan coverage

  private void scanPanel(double x0) {
    double[][] turned = new double[2][2]; // [condition][version]
    double[][] conditions = {{0, 1.0}, {2, 0.7}};
    for (int c = 0; c < 2; c++) {
      KinematicRobot r1 = robot(Vec2.of(0, 0), 0, conditions[c]);
      LegacyV1.rotateAndFindGreen(r1);
      KinematicRobot r2 = robot(Vec2.of(0, 0), 0, conditions[c]);
      handler(r2).scan();
      turned[c][0] = r1.cumulativeTurnDeg;
      turned[c][1] = r2.cumulativeTurnDeg;
    }
    panelTitle(x0, "How far the \"360°\" jar scan turns", "degrees actually rotated");

    double labelW = 120;
    double plotX = x0 + labelW;
    double plotW = PANEL_W - labelW - 10;
    double max = 400;
    String[] names = {"Ideal model", "30% wheel slip"};
    double y = TOP + 20;
    for (int c = 0; c < 2; c++) {
      add("<text x=\"%.1f\" y=\"%.1f\" font-size=\"16\" fill=\"%s\">%s</text>", x0, y + 24, INK_2, names[c]);
      for (int v = 0; v < 2; v++) {
        double barW = plotW * turned[c][v] / max;
        double by = y + v * 24;
        bar(plotX, by, barW, 22, v == 0 ? V1 : V2);
        add("<text x=\"%.1f\" y=\"%.1f\" font-size=\"15\" fill=\"%s\">%.0f°</text>",
            plotX + barW + 6, by + 16, INK, turned[c][v]);
      }
      y += 80;
    }
    double fullX = plotX + plotW * 360 / max;
    add("<line x1=\"%.1f\" y1=\"%.1f\" x2=\"%.1f\" y2=\"%.1f\" stroke=\"%s\" stroke-width=\"1.5\" "
        + "stroke-dasharray=\"5 4\"/>", fullX, TOP + 8, fullX, TOP + 175, INK_2);
    add("<text x=\"%.1f\" y=\"%.1f\" font-size=\"14\" fill=\"%s\" text-anchor=\"middle\">full turn</text>",
        fullX, TOP + 192, INK_2);
    add("<text x=\"%.1f\" y=\"%.1f\" font-size=\"14\" fill=\"%s\">v1 turns for a fixed time;"
        + " v2 measures with the compass.</text>", x0, TOP + PLOT_H + 32, INK_2);
  }

  // ------------------------------------------------------------------ panel 2: drift on a leg

  private void driftPanel(double x0) {
    Vec2 target = Vec2.of(530, 0);
    KinematicRobot r1 = robot(Vec2.of(0, 0), 2.9, new double[] {0, 1.0});
    r1.trace = new ArrayList<>();
    LegacyV1.moveHardcodedAhead(r1, 0, 0, target.x, target.y, 10_000);
    KinematicRobot r2 = robot(Vec2.of(0, 0), 2.9, new double[] {0, 1.0});
    r2.trace = new ArrayList<>();
    new Navigator(r2, new MissionStats(r2::elapsedMs)).driveTo(target);

    panelTitle(x0, "Driving a 530 cm leg that starts 2.9° off", "sideways error (cm) vs distance driven (cm)");
    double plotX = x0 + 40;
    double plotW = PANEL_W - 110;
    double plotTop = TOP + 10;
    double plotH = PLOT_H - 30;
    double maxY = 30;
    for (int g = 0; g <= 30; g += 10) {
      double gy = plotTop + plotH - plotH * g / maxY;
      add("<line x1=\"%.1f\" y1=\"%.1f\" x2=\"%.1f\" y2=\"%.1f\" stroke=\"%s\" stroke-width=\"1\"/>",
          plotX, gy, plotX + plotW, gy, GRID);
      add("<text x=\"%.1f\" y=\"%.1f\" font-size=\"14\" fill=\"%s\" text-anchor=\"end\">%d</text>",
          plotX - 6, gy + 5, INK_2, g);
    }
    for (int d = 0; d <= 500; d += 250) {
      add("<text x=\"%.1f\" y=\"%.1f\" font-size=\"14\" fill=\"%s\" text-anchor=\"middle\">%d</text>",
          plotX + plotW * d / 530.0, plotTop + plotH + 20, INK_2, d);
    }
    Vec2 end1 = line(r1.trace, plotX, plotTop, plotW, plotH, maxY, V1);
    Vec2 end2 = line(r2.trace, plotX, plotTop, plotW, plotH, maxY, V2);
    double off1 = r1.positionCm().distanceTo(target);
    double off2 = r2.positionCm().distanceTo(target);
    add("<text x=\"%.1f\" y=\"%.1f\" font-size=\"15\" fill=\"%s\">%.1f cm off</text>",
        end1.x + 8, end1.y + 5, INK, off1);
    add("<text x=\"%.1f\" y=\"%.1f\" font-size=\"15\" fill=\"%s\">%.1f cm</text>",
        end2.x + 8, end2.y + 5, INK, off2);
    add("<text x=\"%.1f\" y=\"%.1f\" font-size=\"14\" fill=\"%s\">v1 drives open-loop;"
        + " v2 steers towards the waypoint.</text>", x0, TOP + PLOT_H + 32, INK_2);
  }

  /** Plots |y| (sideways error) against x and returns the last point in SVG coordinates. */
  private Vec2 line(List<Vec2> trace, double px, double py, double pw, double ph, double maxY, String color) {
    StringBuilder pts = new StringBuilder();
    double sx = px;
    double sy = py + ph;
    pts.append(String.format(Locale.ROOT, "%.1f,%.1f ", sx, sy));
    for (int i = 0; i < trace.size(); i += 4) {
      Vec2 p = trace.get(i);
      sx = px + pw * Math.min(p.x, 530) / 530.0;
      sy = py + ph - ph * Math.min(Math.abs(p.y), maxY) / maxY;
      pts.append(String.format(Locale.ROOT, "%.1f,%.1f ", sx, sy));
    }
    Vec2 last = trace.get(trace.size() - 1);
    sx = px + pw * Math.min(last.x, 530) / 530.0;
    sy = py + ph - ph * Math.min(Math.abs(last.y), maxY) / maxY;
    pts.append(String.format(Locale.ROOT, "%.1f,%.1f", sx, sy));
    add("<polyline points=\"%s\" fill=\"none\" stroke=\"%s\" stroke-width=\"3\" stroke-linejoin=\"round\" "
        + "stroke-linecap=\"round\"/>", pts, color);
    add("<circle cx=\"%.1f\" cy=\"%.1f\" r=\"5\" fill=\"%s\" stroke=\"%s\" stroke-width=\"2\"/>",
        sx, sy, color, SURFACE);
    return Vec2.of(sx, sy);
  }

  // ------------------------------------------------------------------ panel 3: failed search

  private void retryPanel(double x0) {
    double v1M = Routes.length(Routes.SEARCH_POINT, Routes.DROPOFF_TO_SEARCH) / 100;
    double v2M = Routes.length(Routes.SEARCH_POINT, Routes.TO_SEARCH_POINT) / 100;
    panelTitle(x0, "After a failed search (jars 2-5)", "metres driven before searching again");
    double plotX = x0;
    double plotW = PANEL_W - 70;
    double max = 45;
    double y = TOP + 20;
    double[] values = {v1M, v2M};
    String[] notes = {"via the drop-off, through 4 immovable boxes", "straight back to the search point"};
    for (int v = 0; v < 2; v++) {
      double barW = Math.max(3, plotW * values[v] / max);
      bar(plotX, y, barW, 30, v == 0 ? V1 : V2);
      add("<text x=\"%.1f\" y=\"%.1f\" font-size=\"15\" fill=\"%s\">%.1f m</text>",
          plotX + barW + 6, y + 21, INK, values[v]);
      add("<text x=\"%.1f\" y=\"%.1f\" font-size=\"14\" fill=\"%s\">%s</text>", plotX, y + 52, INK_2, notes[v]);
      y += 85;
    }
  }

  // ------------------------------------------------------------------ helpers

  private void panelTitle(double x, String title, String subtitle) {
    add("<text x=\"%.1f\" y=\"%.1f\" font-size=\"19\" font-weight=\"bold\" fill=\"%s\">%s</text>",
        x, TOP - 32, INK, title);
    add("<text x=\"%.1f\" y=\"%.1f\" font-size=\"15\" fill=\"%s\">%s</text>", x, TOP - 10, INK_2, subtitle);
  }

  private void legendSwatch(double x, double y, String color, String text) {
    add("<rect x=\"%.1f\" y=\"%.1f\" width=\"22\" height=\"14\" rx=\"3\" fill=\"%s\"/>", x, y - 12, color);
    add("<text x=\"%.1f\" y=\"%.1f\" font-size=\"17\" fill=\"%s\">%s</text>", x + 30, y, INK, text);
  }

  private void bar(double x, double y, double w, double h, String color) {
    add("<rect x=\"%.1f\" y=\"%.1f\" width=\"%.1f\" height=\"%.1f\" rx=\"4\" fill=\"%s\"/>", x, y, w, h, color);
  }

  private static KinematicRobot robot(Vec2 p, double heading, double[] imperfections) {
    return new KinematicRobot(p, heading).withImperfections((int) imperfections[0], imperfections[1]);
  }

  private static JarHandler handler(KinematicRobot robot) {
    MissionStats stats = new MissionStats(robot::elapsedMs);
    return new JarHandler(robot, new Navigator(robot, stats), stats, new Random(1));
  }

  private void add(String format, Object... args) {
    svg.append(String.format(Locale.ROOT, format, args)).append('\n');
  }
}
