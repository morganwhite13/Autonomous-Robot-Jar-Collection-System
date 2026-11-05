import static org.junit.jupiter.api.Assertions.assertEquals;
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
    RoutePlanner planner = RoutePlanner.forWorld(world, true, RoutePlannerReport.CLEARANCE_CM);
    assertTrue(planner.plan(Routes.SEARCH_POINT, Routes.DROPOFF_ENTRY).isEmpty());
  }

  @Test
  void plannedCarryRouteIsShorterAndKeepsClearOfStaticObstacles() {
    RoutePlanner planner = RoutePlanner.afterClearing(world, RoutePlannerReport.CLEARANCE_CM,
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
}
