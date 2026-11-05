import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Hand-tuned waypoints, in cm (see {@link Vec2} for the frame).
 *
 * <p>The coordinates are exactly the ones v1 used; {@code RoutesTest} checks them against the
 * original expressions. In v1 they were rebuilt inline in five different methods, which made
 * them easy to edit inconsistently; here each route is defined once and named.
 */
final class Routes {
  private Routes() {}

  /** Where every jar search starts: just north of the opening into the jar bay. */
  static final Vec2 SEARCH_POINT = Vec2.of(850, 60);
  /** Entry point of the drop-off lane in front of the five wooden pads. */
  static final Vec2 DROPOFF_ENTRY = Vec2.of(-850, 602);

  /**
   * Phase 1: push cardboard boxes out of the way, ending at the search point.
   * v1 wrote these as offsets of a 35 cm buffer ({@code bffr}) from box edges.
   */
  static final List<Vec2> CLEARING_PATH = List.of(
      Vec2.of(-885, 530),
      Vec2.of(-875, 800),
      Vec2.of(-814, 555),
      Vec2.of(-814, 375),
      Vec2.of(-566, 375),
      Vec2.of(-506, 800),
      Vec2.of(-210, 497),
      Vec2.of(-210, 122),
      Vec2.of(-210, 278),
      Vec2.of(320, 278),
      Vec2.of(320, 95),
      Vec2.of(478, 144),
      Vec2.of(876, 144),
      Vec2.of(875, 100),
      SEARCH_POINT);

  /** Short hop back to the search point from anywhere near the jar bay. */
  static final List<Vec2> TO_SEARCH_POINT = List.of(Vec2.of(850, 100), SEARCH_POINT);

  /** Carry a jar from the jar bay to the drop-off lane. */
  static final List<Vec2> SEARCH_TO_DROPOFF = List.of(
      Vec2.of(850, 100),
      Vec2.of(350, 144),
      Vec2.of(0, 144),
      Vec2.of(-75, 278),
      Vec2.of(-215, 278),
      Vec2.of(-340, 605),
      Vec2.of(-590, 510),
      Vec2.of(-590, 390),
      Vec2.of(-815, 410),
      DROPOFF_ENTRY);

  /**
   * Return from the drop-off lane to the search point. Not an exact reverse of
   * {@link #SEARCH_TO_DROPOFF}: v1 added (-220, 400) on the way back, kept here unchanged.
   */
  static final List<Vec2> DROPOFF_TO_SEARCH = List.of(
      DROPOFF_ENTRY,
      Vec2.of(-815, 410),
      Vec2.of(-590, 390),
      Vec2.of(-590, 510),
      Vec2.of(-340, 605),
      Vec2.of(-220, 400),
      Vec2.of(-215, 278),
      Vec2.of(-75, 278),
      Vec2.of(0, 144),
      Vec2.of(350, 144),
      Vec2.of(850, 100),
      SEARCH_POINT);

  /**
   * Opt-in alternative to the carry and return routes ({@link Config#USE_PLANNED_ROUTES}):
   * shortest routes found by the A* planner in tools/RoutePlanner.java, keeping the same 35 cm
   * clearance, assuming the boxes the clearing route sweeps past end up out of the way. The
   * carry route still leaves the jar bay through (850, 100). About 13-14% shorter than the
   * hand-tuned routes; in Webots the mission finished 10% sooner on them. RoutePlannerTest checks
   * these still match the planner.
   */
  static final List<Vec2> PLANNED_SEARCH_TO_DROPOFF = List.of(
      Vec2.of(850, 100),
      Vec2.of(393, 153),
      Vec2.of(-197, 313),
      Vec2.of(-387, 543),
      Vec2.of(-422, 548),
      Vec2.of(-497, 548),
      Vec2.of(-647, 403),
      Vec2.of(-732, 403),
      Vec2.of(-757, 423),
      DROPOFF_ENTRY);

  static final List<Vec2> PLANNED_DROPOFF_TO_SEARCH = List.of(
      DROPOFF_ENTRY,
      Vec2.of(-752, 418),
      Vec2.of(-727, 403),
      Vec2.of(-632, 403),
      Vec2.of(-487, 543),
      Vec2.of(-462, 548),
      Vec2.of(-392, 548),
      Vec2.of(-197, 313),
      Vec2.of(408, 153),
      Vec2.of(783, 108),
      SEARCH_POINT);

  /** The pair of routes a mission uses to carry jars and come back for the next one. */
  static final class RouteSet {
    final String name;
    final List<Vec2> searchToDropoff;
    final List<Vec2> dropoffToSearch;

    RouteSet(String name, List<Vec2> searchToDropoff, List<Vec2> dropoffToSearch) {
      this.name = name;
      this.searchToDropoff = searchToDropoff;
      this.dropoffToSearch = dropoffToSearch;
    }
  }

  static final RouteSet HAND_TUNED =
      new RouteSet("hand-tuned", SEARCH_TO_DROPOFF, DROPOFF_TO_SEARCH);
  static final RouteSet PLANNED =
      new RouteSet("planned (A*)", PLANNED_SEARCH_TO_DROPOFF, PLANNED_DROPOFF_TO_SEARCH);

  /** Lane spacing between drop-off slots, cm (v1). */
  static final int SLOT_SPACING_CM = 30;

  /** y coordinate of the drop-off slot for the jar with 0-based index {@code jarIndex}. */
  static int slotY(int jarIndex) {
    return 602 + jarIndex * SLOT_SPACING_CM + 5;
  }

  /** v1 placeJar path: line up with the slot, then creep forward to the pad in three short legs. */
  static List<Vec2> placementPath(int jarIndex) {
    int y = slotY(jarIndex);
    List<Vec2> path = new ArrayList<>();
    path.add(Vec2.of(-850, y));
    path.add(Vec2.of(-900, y));
    path.add(Vec2.of(-915, y));
    path.add(Vec2.of(-936, y));
    return Collections.unmodifiableList(path);
  }

  /** Total length of a route when starting from {@code start}, in cm. */
  static double length(Vec2 start, List<Vec2> route) {
    double total = 0;
    Vec2 here = start;
    for (Vec2 next : route) {
      total += here.distanceTo(next);
      here = next;
    }
    return total;
  }
}
