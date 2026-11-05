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
