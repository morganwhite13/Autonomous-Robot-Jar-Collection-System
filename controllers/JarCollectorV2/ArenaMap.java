import java.util.List;

/**
 * What v2 knows about the arena before it starts: the footprints of the walls and the wooden
 * boxes that can't be pushed (not the cardboard boxes, which move, or the drop-off pads, which
 * the robot drives up to). Corners in cm, in the controllers' frame. Copied from
 * worlds/ProjectWorld2025.wbt; ArenaMapTest checks they still match it.
 */
final class ArenaMap {
  private ArenaMap() {}

  /** Keep the robot's centre this far from walls and boxes (it is 38 cm wide, 45 cm long). */
  static final double CLEARANCE_CM = 30;
  /** Planner grid cell size. */
  static final double CELL_CM = 5;

  static final List<Vec2[]> STATIC_OBSTACLES = List.of(
      box(-471.0, 308.0, -271.0, 308.0, -271.0, 248.0, -471.0, 248.0), // wooden box(2)
      box(-471.0, 308.0, -471.0, 508.0, -411.0, 508.0, -411.0, 308.0), // wooden box(1)
      box(-115.0, 222.0, -115.0, 22.0, -175.0, 22.0, -175.0, 222.0), // wooden box(14)
      box(-115.0, 547.0, -115.0, 347.0, -175.0, 347.0, -175.0, 547.0), // wooden box(15)
      box(298.0, 588.0, 298.0, 388.0, 238.0, 388.0, 238.0, 588.0), // wooden box(3)
      box(-29.0, 688.0, 171.0, 688.0, 171.0, 628.0, -29.0, 628.0), // wooden box(4)
      box(-238.0, 679.0, -238.0, 979.0, -178.0, 979.0, -178.0, 679.0), // wooden box(11)
      box(-738.0, 510.0, -738.0, 810.0, -678.0, 810.0, -678.0, 510.0), // wooden box(12)
      box(-934.0, 220.0, -634.0, 220.0, -634.0, 160.0, -934.0, 160.0), // wooden box(13)
      box(415.0, 188.0, 415.0, 388.0, 715.0, 388.0, 715.0, 188.0), // wooden box(5)
      box(-11.0, -890.0, -11.0, -10.0, 9.0, -10.0, 9.0, -890.0), // wall(1)
      box(-1000.0, -999.0, -1000.0, 1001.0, -980.0, 1001.0, -980.0, -999.0), // wall(18)
      box(981.0, -1000.0, 981.0, 1000.0, 1001.0, 1000.0, 1001.0, -1000.0), // wall(5)
      box(-980.0, 992.0, 980.0, 992.0, 980.0, 972.0, -980.0, 972.0), // wall(7)
      box(-980.0, -990.0, 980.0, -990.0, 980.0, -1010.0, -980.0, -1010.0), // wall(4)
      box(-980.0, -10.0, -20.0, -10.0, -20.0, -30.0, -980.0, -30.0), // wall(17)
      box(-15.0, -10.0, 785.0, -10.0, 785.0, -30.0, -15.0, -30.0), // wall(6)
      box(581.0, -220.0, 981.0, -220.0, 981.0, -240.0, 581.0, -240.0), // wall(2)
      box(198.0, -656.0, 198.0, -256.0, 218.0, -256.0, 218.0, -656.0), // wall(8)
      box(-15.0, -980.0, -15.0, -880.0, -10.0, -880.0, -10.0, -980.0), // wall(19)
      box(237.0, -979.0, 137.0, -979.0, 137.0, -974.0, 237.0, -974.0), // wall(3)
      box(979.0, -180.0, 979.0, -80.0, 984.0, -80.0, 984.0, -180.0), // wall(21)
      box(-9.0, -980.0, -9.0, -880.0, -4.0, -880.0, -4.0, -980.0), // wall(20)
      box(208.0, -646.0, 608.0, -646.0, 608.0, -666.0, 208.0, -666.0), // wall(9)
      box(583.0, -802.0, 783.0, -802.0, 783.0, -822.0, 583.0, -822.0), // wall(14)
      box(326.0, -431.0, 826.0, -431.0, 826.0, -451.0, 326.0, -451.0), // wall(12)
      box(378.0, -583.0, 378.0, -983.0, 358.0, -983.0, 358.0, -583.0), // wall(10)
      box(792.0, -440.0, 792.0, -840.0, 772.0, -840.0, 772.0, -440.0), // wall(13)
      box(379.0, -21.0, 379.0, -421.0, 359.0, -421.0, 359.0, -21.0), // wall(11)
      box(785.0, -20.0, 785.0, -48.0, 765.0, -48.0, 765.0, -20.0)  // wall(15)
  );

  /** A planner over the static obstacles plus {@code extra} ones (such as something that blocked the robot). */
  static RoutePlanner planner(List<Vec2[]> extra) {
    List<Vec2[]> all = new java.util.ArrayList<>(STATIC_OBSTACLES);
    all.addAll(extra);
    return new RoutePlanner(all, CLEARANCE_CM, CELL_CM, -1000, -1000, 1000, 1000);
  }

  private static Vec2[] box(double x0, double y0, double x1, double y1, double x2, double y2,
      double x3, double y3) {
    return new Vec2[] {Vec2.of(x0, y0), Vec2.of(x1, y1), Vec2.of(x2, y2), Vec2.of(x3, y3)};
  }
}
