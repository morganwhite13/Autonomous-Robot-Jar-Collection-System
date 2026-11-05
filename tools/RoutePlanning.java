import java.util.ArrayList;
import java.util.List;

/** {@link RoutePlanner}s built from a world file, for the route analysis and its tests. */
final class RoutePlanning {
  private RoutePlanning() {}

  /** Planner for a world file: walls and wooden boxes, plus cardboard boxes if requested. */
  static RoutePlanner forWorld(WorldFile world, boolean cardboardIsObstacle, double clearanceCm) {
    List<Vec2[]> polygons = new ArrayList<>();
    for (WorldFile.Obstacle o : world.obstacles) {
      if (o.isStatic() || (cardboardIsObstacle && o.kind == WorldFile.Kind.CARDBOARD_BOX)) {
        polygons.add(o.corners);
      }
    }
    return new RoutePlanner(polygons, clearanceCm, 5, -1000, -1000, 1000, 1000);
  }

  /**
   * Cardboard boxes the box-clearing route passes within {@code sweepCm} of. These are assumed to
   * be pushed out of the way; the others are assumed to stay where they started.
   */
  static List<WorldFile.Obstacle> sweptByClearing(WorldFile world, double sweepCm) {
    List<Vec2> route = new ArrayList<>();
    route.add(world.robotStart);
    route.addAll(Routes.CLEARING_PATH);
    List<WorldFile.Obstacle> swept = new ArrayList<>();
    for (WorldFile.Obstacle box : world.ofKind(WorldFile.Kind.CARDBOARD_BOX)) {
      for (int i = 0; i + 1 < route.size(); i++) {
        if (WorldFile.clearance(route.get(i), route.get(i + 1), box) <= sweepCm) {
          swept.add(box);
          break;
        }
      }
    }
    return swept;
  }

  /** Planner for the state after the clearing phase: static obstacles plus unswept boxes. */
  static RoutePlanner afterClearing(WorldFile world, double clearanceCm, double sweepCm) {
    List<WorldFile.Obstacle> swept = sweptByClearing(world, sweepCm);
    List<Vec2[]> polygons = new ArrayList<>();
    for (WorldFile.Obstacle o : world.obstacles) {
      boolean unsweptBox = o.kind == WorldFile.Kind.CARDBOARD_BOX && !swept.contains(o);
      if (o.isStatic() || unsweptBox) {
        polygons.add(o.corners);
      }
    }
    return new RoutePlanner(polygons, clearanceCm, 5, -1000, -1000, 1000, 1000);
  }
}
