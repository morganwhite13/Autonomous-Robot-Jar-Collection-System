import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.PriorityQueue;

/**
 * A* path planner on an occupancy grid. v2 uses it to find a way around something that stopped
 * it ({@link Navigator}), on the map of walls and immovable boxes in {@link ArenaMap}. The tools
 * also use it to compare the hand-tuned routes with the shortest routes that keep the same
 * clearance (tools/RoutePlanning.java, docs/route-planner.md).
 *
 * <p>Obstacles are footprint polygons in the controllers' cm frame. A grid cell is blocked if
 * its centre is within {@code clearanceCm} of any obstacle. The raw 8-connected A* path is then
 * shortened by "string pulling": each waypoint jumps to the furthest later point it can reach
 * in a straight line through free cells.
 */
final class RoutePlanner {

  private final double minX;
  private final double minY;
  private final double cell;
  private final int cols;
  private final int rows;
  private final boolean[] blocked;

  RoutePlanner(List<Vec2[]> obstacles, double clearanceCm, double cellCm,
      double minX, double minY, double maxX, double maxY) {
    this.minX = minX;
    this.minY = minY;
    this.cell = cellCm;
    this.cols = (int) Math.ceil((maxX - minX) / cellCm);
    this.rows = (int) Math.ceil((maxY - minY) / cellCm);
    this.blocked = new boolean[cols * rows];
    for (Vec2[] polygon : obstacles) {
      markBlocked(polygon, clearanceCm);
    }
  }

  private void markBlocked(Vec2[] polygon, double clearance) {
    double lx = Double.MAX_VALUE;
    double ly = Double.MAX_VALUE;
    double hx = -Double.MAX_VALUE;
    double hy = -Double.MAX_VALUE;
    for (Vec2 p : polygon) {
      lx = Math.min(lx, p.x);
      ly = Math.min(ly, p.y);
      hx = Math.max(hx, p.x);
      hy = Math.max(hy, p.y);
    }
    int c0 = Math.max(0, col(lx - clearance));
    int c1 = Math.min(cols - 1, col(hx + clearance));
    int r0 = Math.max(0, row(ly - clearance));
    int r1 = Math.min(rows - 1, row(hy + clearance));
    for (int r = r0; r <= r1; r++) {
      for (int c = c0; c <= c1; c++) {
        Vec2 centre = centre(c, r);
        if (distanceToPolygon(centre, polygon) <= clearance) {
          blocked[r * cols + c] = true;
        }
      }
    }
  }

  static double distanceToPolygon(Vec2 p, Vec2[] polygon) {
    if (contains(polygon, p)) {
      return 0;
    }
    double best = Double.MAX_VALUE;
    for (int i = 0; i < polygon.length; i++) {
      best = Math.min(best, pointToSegment(p, polygon[i], polygon[(i + 1) % polygon.length]));
    }
    return best;
  }

  /** Point-in-polygon by ray casting. */
  static boolean contains(Vec2[] polygon, Vec2 p) {
    boolean inside = false;
    for (int i = 0, j = polygon.length - 1; i < polygon.length; j = i++) {
      Vec2 a = polygon[i];
      Vec2 b = polygon[j];
      if ((a.y > p.y) != (b.y > p.y) && p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x) {
        inside = !inside;
      }
    }
    return inside;
  }

  static double pointToSegment(Vec2 p, Vec2 a, Vec2 b) {
    double dx = b.x - a.x;
    double dy = b.y - a.y;
    double len2 = dx * dx + dy * dy;
    double t = len2 == 0 ? 0 : Math.max(0, Math.min(1, ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2));
    return p.distanceTo(Vec2.of(a.x + t * dx, a.y + t * dy));
  }

  /**
   * The centre of the free cell nearest to {@code p}, searching outwards up to {@code maxCm};
   * {@code p} itself if its cell is free, or null if none is that close.
   */
  Vec2 nearestFree(Vec2 p, double maxCm) {
    if (isFree(p)) {
      return p;
    }
    int c0 = col(p.x);
    int r0 = row(p.y);
    int reach = (int) Math.ceil(maxCm / cell);
    Vec2 best = null;
    double bestDistance = maxCm;
    for (int r = r0 - reach; r <= r0 + reach; r++) {
      for (int c = c0 - reach; c <= c0 + reach; c++) {
        if (c < 0 || r < 0 || c >= cols || r >= rows || blocked[r * cols + c]) {
          continue;
        }
        Vec2 centre = centre(c, r);
        double d = centre.distanceTo(p);
        if (d <= bestDistance) {
          bestDistance = d;
          best = centre;
        }
      }
    }
    return best;
  }

  boolean isFree(Vec2 p) {
    int c = col(p.x);
    int r = row(p.y);
    return c >= 0 && r >= 0 && c < cols && r < rows && !blocked[r * cols + c];
  }

  /**
   * Shortest route from {@code start} to {@code goal} as a list of waypoints (excluding the
   * start, like the hand-tuned routes), or an empty list if the goal can't be reached.
   */
  List<Vec2> plan(Vec2 start, Vec2 goal) {
    int s = index(start);
    int g = index(goal);
    if (s < 0 || g < 0 || blocked[s] || blocked[g]) {
      return Collections.emptyList();
    }
    double[] cost = new double[cols * rows];
    Arrays.fill(cost, Double.MAX_VALUE);
    int[] parent = new int[cols * rows];
    Arrays.fill(parent, -1);
    boolean[] closed = new boolean[cols * rows];
    PriorityQueue<double[]> open = new PriorityQueue<>((a, b) -> Double.compare(a[0], b[0]));
    cost[s] = 0;
    open.add(new double[] {heuristic(s, g), s});
    int[][] moves = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
    while (!open.isEmpty()) {
      int current = (int) open.poll()[1];
      if (closed[current]) {
        continue;
      }
      if (current == g) {
        break;
      }
      closed[current] = true;
      int cc = current % cols;
      int cr = current / cols;
      for (int[] m : moves) {
        int nc = cc + m[0];
        int nr = cr + m[1];
        if (nc < 0 || nr < 0 || nc >= cols || nr >= rows) {
          continue;
        }
        int next = nr * cols + nc;
        // Diagonal moves may not cut the corner of a blocked cell.
        if (blocked[next] || (m[0] != 0 && m[1] != 0
            && (blocked[cr * cols + nc] || blocked[nr * cols + cc]))) {
          continue;
        }
        double step = (m[0] != 0 && m[1] != 0) ? Math.sqrt(2) : 1;
        double candidate = cost[current] + step;
        if (candidate < cost[next]) {
          cost[next] = candidate;
          parent[next] = current;
          open.add(new double[] {candidate + heuristic(next, g), next});
        }
      }
    }
    if (parent[g] < 0 && g != s) {
      return Collections.emptyList();
    }
    List<Vec2> raw = new ArrayList<>();
    for (int i = g; i != s; i = parent[i]) {
      raw.add(centre(i % cols, i / cols));
    }
    raw.add(start);
    Collections.reverse(raw);
    raw.set(raw.size() - 1, goal);
    return smooth(raw);
  }

  /**
   * String pulling: from each waypoint, jump to the furthest point still in line of sight. Then
   * drop any waypoint closer than {@link #MIN_LEG_CM} to the next one when the robot can go
   * straight past it, since each extra waypoint costs a turn on the spot.
   */
  private List<Vec2> smooth(List<Vec2> raw) {
    List<Vec2> pulled = new ArrayList<>();
    pulled.add(raw.get(0));
    int i = 0;
    while (i < raw.size() - 1) {
      int j = raw.size() - 1;
      while (j > i + 1 && !lineOfSight(raw.get(i), raw.get(j))) {
        j--;
      }
      pulled.add(raw.get(j));
      i = j;
    }
    for (int k = pulled.size() - 2; k >= 1; k--) {
      Vec2 here = pulled.get(k);
      if (here.distanceTo(pulled.get(k + 1)) < MIN_LEG_CM
          && lineOfSight(pulled.get(k - 1), pulled.get(k + 1))) {
        pulled.remove(k);
      }
    }
    return pulled.subList(1, pulled.size());
  }

  /** Legs shorter than this are merged into their neighbours where line of sight allows. */
  static final double MIN_LEG_CM = 20;

  /** True if every point along the segment (sampled every half cell) is in a free cell. */
  boolean lineOfSight(Vec2 a, Vec2 b) {
    int samples = (int) Math.ceil(a.distanceTo(b) / (cell / 2)) + 1;
    for (int k = 0; k <= samples; k++) {
      double t = (double) k / samples;
      if (!isFree(Vec2.of(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)))) {
        return false;
      }
    }
    return true;
  }

  private double heuristic(int from, int to) {
    double dx = Math.abs(from % cols - to % cols);
    double dy = Math.abs(from / cols - to / cols);
    return Math.max(dx, dy) + (Math.sqrt(2) - 1) * Math.min(dx, dy); // octile distance
  }

  private int col(double x) {
    return (int) Math.floor((x - minX) / cell);
  }

  private int row(double y) {
    return (int) Math.floor((y - minY) / cell);
  }

  private int index(Vec2 p) {
    int c = col(p.x);
    int r = row(p.y);
    return (c < 0 || r < 0 || c >= cols || r >= rows) ? -1 : r * cols + c;
  }

  private Vec2 centre(int c, int r) {
    return Vec2.of(minX + (c + 0.5) * cell, minY + (r + 0.5) * cell);
  }
}
