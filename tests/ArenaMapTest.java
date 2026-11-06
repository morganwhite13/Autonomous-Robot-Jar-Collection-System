import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

class ArenaMapTest {

  @Test
  void staticObstaclesMatchTheWorldFile() throws Exception {
    WorldFile world = WorldFile.read(Path.of("worlds/ProjectWorld2025.wbt"));
    List<WorldFile.Obstacle> expected = new ArrayList<>();
    for (WorldFile.Obstacle o : world.obstacles) {
      if (o.isStatic() && !o.isDropoffPad()) {
        expected.add(o);
      }
    }
    assertEquals(expected.size(), ArenaMap.STATIC_OBSTACLES.size());
    for (int i = 0; i < expected.size(); i++) {
      Vec2[] polygon = ArenaMap.STATIC_OBSTACLES.get(i);
      for (int k = 0; k < 4; k++) {
        assertTrue(polygon[k].distanceTo(expected.get(i).corners[k]) < 0.1,
            expected.get(i).name + " corner " + k);
      }
    }
  }

  @Test
  void theRoutesOwnWaypointsAreReachableOnTheMap() {
    RoutePlanner planner = ArenaMap.planner(List.of());
    // The search point and the first carry waypoint are in open floor.
    assertTrue(planner.isFree(Routes.SEARCH_POINT));
    assertFalse(planner.plan(Routes.SEARCH_POINT, Routes.SEARCH_TO_DROPOFF.get(4)).isEmpty());
  }
}
