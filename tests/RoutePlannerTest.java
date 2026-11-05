import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.nio.file.Path;
import java.util.List;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class RoutePlannerTest {

  private static WorldFile world;

  @BeforeAll
  static void loadWorld() throws Exception {
    world = WorldFile.read(Path.of("worlds/ProjectWorld2025.wbt"));
  }

  private static Vec2[] square(double cx, double cy, double half) {
    return new Vec2[] {Vec2.of(cx - half, cy - half), Vec2.of(cx + half, cy - half),
        Vec2.of(cx + half, cy + half), Vec2.of(cx - half, cy + half)};
  }

  @Test
  void emptySpaceGivesOneStraightLeg() {
    RoutePlanner planner = new RoutePlanner(List.of(), 35, 5, -500, -500, 500, 500);
    List<Vec2> route = planner.plan(Vec2.of(0, 0), Vec2.of(300, 400));
    assertEquals(1, route.size());
    assertEquals(500, Routes.length(Vec2.of(0, 0), route), 1e-9);
  }

  @Test
  void goesAroundAnObstacleKeepingItsClearance() {
    Vec2[] wall = {Vec2.of(-20, -200), Vec2.of(20, -200), Vec2.of(20, 200), Vec2.of(-20, 200)};
    RoutePlanner planner = new RoutePlanner(List.<Vec2[]>of(wall), 35, 5, -500, -500, 500, 500);
    Vec2 start = Vec2.of(-200, 0);
    List<Vec2> route = planner.plan(start, Vec2.of(200, 0));
    assertTrue(route.size() >= 2, "must detour: " + route);
    Vec2 here = start;
    for (Vec2 next : route) {
      double clearance = WorldFile.clearance(here, next, new WorldFile.Obstacle(
          WorldFile.Kind.WALL, "wall", Vec2.of(0, 0), wall, false, 0));
      assertTrue(clearance >= 35 - 5, here + " -> " + next + " clears by " + clearance);
      here = next;
    }
  }

  @Test
  void unreachableGoalGivesNoRoute() {
    RoutePlanner planner = new RoutePlanner(List.<Vec2[]>of(square(100, 0, 50)), 35, 5, -500, -500, 500, 500);
    assertTrue(planner.plan(Vec2.of(-200, 0), Vec2.of(100, 0)).isEmpty());
  }

  @Test
  void storedPlannedRoutesStillMatchThePlanner() {
    assertEquals(RoutePlannerReport.plannedCarry(world), Routes.PLANNED_SEARCH_TO_DROPOFF);
    assertEquals(RoutePlannerReport.plannedReturn(world), Routes.PLANNED_DROPOFF_TO_SEARCH);
  }

  @Test
  void withoutTheClearingPhaseThereIsNoRoute() {
    RoutePlanner planner = RoutePlanning.forWorld(world, true, RoutePlannerReport.CLEARANCE_CM);
    assertTrue(planner.plan(Routes.SEARCH_POINT, Routes.DROPOFF_ENTRY).isEmpty());
  }

  @Test
  void plannedCarryRouteIsShorterAndKeepsClearOfStaticObstacles() {
    RoutePlanner planner = RoutePlanning.afterClearing(world, RoutePlannerReport.CLEARANCE_CM,
        RoutePlannerReport.SWEEP_CM);
    List<Vec2> route = planner.plan(Routes.SEARCH_POINT, Routes.DROPOFF_ENTRY);
    assertEquals(Routes.DROPOFF_ENTRY, route.get(route.size() - 1));
    double planned = Routes.length(Routes.SEARCH_POINT, route);
    double handTuned = Routes.length(Routes.SEARCH_POINT, Routes.SEARCH_TO_DROPOFF);
    assertTrue(planned < handTuned, planned + " vs " + handTuned);
    Vec2 here = Routes.SEARCH_POINT;
    for (Vec2 next : route) {
      for (WorldFile.Obstacle o : world.staticObstacles()) {
        assertTrue(WorldFile.clearance(here, next, o) >= 30, here + " -> " + next + " near " + o.name);
      }
      here = next;
    }
  }

  @Test
  void pointsInsideAndOutsideAPolygon() {
    Vec2[] triangle = {Vec2.of(0, 0), Vec2.of(100, 0), Vec2.of(0, 100)};
    assertTrue(RoutePlanner.contains(triangle, Vec2.of(10, 10)));
    assertTrue(RoutePlanner.contains(triangle, Vec2.of(49, 49)));
    assertTrue(RoutePlanner.contains(triangle, Vec2.of(1, 98)));
    assertFalse(RoutePlanner.contains(triangle, Vec2.of(51, 51))); // just past the long side
    assertFalse(RoutePlanner.contains(triangle, Vec2.of(-1, 50)));
    assertFalse(RoutePlanner.contains(triangle, Vec2.of(150, 50)));
    assertFalse(RoutePlanner.contains(triangle, Vec2.of(20, -1)));
    assertFalse(RoutePlanner.contains(triangle, Vec2.of(20, 101)));
    assertEquals(0, RoutePlanner.distanceToPolygon(Vec2.of(10, 10), triangle), 1e-9);
    assertEquals(5, RoutePlanner.distanceToPolygon(Vec2.of(50, -5), triangle), 1e-9);
    assertEquals(Math.sqrt(50), RoutePlanner.distanceToPolygon(Vec2.of(55, 55), triangle), 1e-9);
    assertEquals(5, RoutePlanner.pointToSegment(Vec2.of(-5, 0), Vec2.of(0, 0), Vec2.of(0, 0)), 1e-9);
  }

  @Test
  void theGridCoversExactlyTheRequestedArea() {
    // 100 x 50 cm in 10 cm cells: 10 columns, 5 rows.
    RoutePlanner planner = new RoutePlanner(List.of(), 0, 10, 0, 0, 100, 50);
    assertTrue(planner.isFree(Vec2.of(0, 0)));
    assertTrue(planner.isFree(Vec2.of(99.9, 49.9)));
    assertFalse(planner.isFree(Vec2.of(-0.1, 10)));
    assertFalse(planner.isFree(Vec2.of(10, -0.1)));
    assertFalse(planner.isFree(Vec2.of(100, 10)));
    assertFalse(planner.isFree(Vec2.of(10, 50)));
    assertEquals(List.of(), planner.plan(Vec2.of(-5, 5), Vec2.of(50, 5)), "start off the grid");
    assertEquals(List.of(), planner.plan(Vec2.of(5, 5), Vec2.of(50, 55)), "goal off the grid");
  }

  @Test
  void plansAlongTheEdgesOfTheGrid() {
    // A wall across the middle leaves only the top row free: the search must explore the last
    // row and column without stepping off the grid.
    Vec2[] wall = {Vec2.of(40, -10), Vec2.of(60, -10), Vec2.of(60, 89), Vec2.of(40, 89)};
    RoutePlanner planner = new RoutePlanner(List.<Vec2[]>of(wall), 0, 10, 0, 0, 100, 100);
    List<Vec2> route = planner.plan(Vec2.of(95, 5), Vec2.of(5, 5));
    assertFalse(route.isEmpty());
    assertEquals(Vec2.of(5, 5), route.get(route.size() - 1));
    for (Vec2 p : route) {
      assertTrue(p.y >= 0 && p.y < 100 && p.x >= 0 && p.x < 100, p.toString());
    }
    assertTrue(route.stream().anyMatch(p -> p.y > 89), "goes over the wall's top: " + route);
  }

  @Test
  void noRouteWhenTheGoalIsWalledOffOrBlocked() {
    Vec2[] wall = {Vec2.of(40, -10), Vec2.of(60, -10), Vec2.of(60, 110), Vec2.of(40, 110)};
    RoutePlanner planner = new RoutePlanner(List.<Vec2[]>of(wall), 0, 10, 0, 0, 100, 100);
    assertEquals(List.of(), planner.plan(Vec2.of(5, 5), Vec2.of(95, 95)));
    assertEquals(List.of(), planner.plan(Vec2.of(5, 5), Vec2.of(50, 50)), "goal inside the wall");
    assertEquals(List.of(), planner.plan(Vec2.of(50, 50), Vec2.of(5, 5)), "start inside the wall");
    assertEquals(List.of(), planner.plan(Vec2.of(5, 5), Vec2.of(5, 5)), "already there");
  }

  @Test
  void nearestFreeFindsTheClosestOpenCell() {
    // A 60 cm square blocks the cells around (0, 0); the closest free cell centres are 35 cm
    // from the middle along each axis.
    RoutePlanner planner = new RoutePlanner(List.<Vec2[]>of(square(0, 0, 30)), 0, 10,
        -100, -100, 100, 100);
    Vec2 free = planner.nearestFree(Vec2.of(2, 0), 80);
    assertTrue(planner.isFree(free));
    assertEquals(Vec2.of(35, 5).distanceTo(Vec2.of(2, 0)), free.distanceTo(Vec2.of(2, 0)), 1e-9,
        free.toString());
    assertEquals(Vec2.of(-80, 50), planner.nearestFree(Vec2.of(-80, 50), 10), "already free");
    assertEquals(null, planner.nearestFree(Vec2.of(0, 0), 30), "nothing within 30 cm");
    // Exactly at the edge of the search: from the middle the closest free centres, such as
    // (35, 5), are sqrt(35^2 + 5^2) away, and a search that far still finds them.
    double edge = Math.hypot(35, 5);
    Vec2 found = planner.nearestFree(Vec2.of(0, 0), edge);
    assertEquals(edge, found.distanceTo(Vec2.of(0, 0)), 1e-9, String.valueOf(found));
  }
}
