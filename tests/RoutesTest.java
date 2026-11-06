import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class RoutesTest {

  private static WorldFile world;

  @BeforeAll
  static void loadWorld() throws Exception {
    world = WorldFile.read(Path.of("worlds/ProjectWorld2025.wbt"));
  }

  @Test
  void waypointsAreExactlyV1s() {
    assertEquals(LegacyV1.clearingPath(), Routes.CLEARING_PATH);
    assertEquals(LegacyV1.navigateToSearchArea(), Routes.TO_SEARCH_POINT);
    assertEquals(LegacyV1.navigateToDropoff(), Routes.SEARCH_TO_DROPOFF);
    assertEquals(LegacyV1.navigateFromDropoffToSearch(), Routes.DROPOFF_TO_SEARCH);
    for (int jarIndex = 0; jarIndex < Config.JAR_COUNT; jarIndex++) {
      assertEquals(LegacyV1.placeJarPath(Config.JAR_COUNT - jarIndex), Routes.placementPath(jarIndex));
    }
  }

  @Test
  void routeLengthAddsUpTheLegsFromTheStart() {
    assertEquals(80, Routes.length(Routes.SEARCH_POINT, Routes.TO_SEARCH_POINT), 1e-9);
    assertEquals(0, Routes.length(Routes.SEARCH_POINT, List.of()), 1e-9);
  }

  @Test
  void worldFileHasTheExpectedContents() {
    assertEquals(20, world.ofKind(WorldFile.Kind.WALL).size());
    assertEquals(15, world.ofKind(WorldFile.Kind.WOODEN_BOX).size());
    assertEquals(32, world.ofKind(WorldFile.Kind.CARDBOARD_BOX).size());
    assertEquals(5, world.ofKind(WorldFile.Kind.HONEY_JAR).size());
    assertTrue(world.robotStart.distanceTo(Vec2.of(-815.65, 556.873)) < 0.01);
  }

  @Test
  void eachDropoffSlotLinesUpWithItsOwnPad() {
    List<WorldFile.Obstacle> pads = new ArrayList<>();
    for (WorldFile.Obstacle o : world.ofKind(WorldFile.Kind.WOODEN_BOX)) {
      if (o.isDropoffPad()) {
        pads.add(o);
      }
    }
    assertEquals(Config.JAR_COUNT, pads.size());
    for (int jarIndex = 0; jarIndex < Config.JAR_COUNT; jarIndex++) {
      int slotY = Routes.slotY(jarIndex);
      long matches = pads.stream().filter(p -> Math.abs(p.center.y - slotY) < 10).count();
      assertEquals(1, matches, "slot " + jarIndex + " at y=" + slotY);
    }
  }

  @Test
  void routesKeepClearOfWallsAndStaticBoxes() {
    assertRouteClearance(world.robotStart, Routes.CLEARING_PATH, 30);
    assertRouteClearance(Routes.SEARCH_POINT, Routes.TO_SEARCH_POINT, 30);
    assertRouteClearance(Routes.SEARCH_POINT, Routes.SEARCH_TO_DROPOFF, 30);
    assertRouteClearance(Routes.DROPOFF_ENTRY, Routes.DROPOFF_TO_SEARCH, 30);
    assertRouteClearance(Routes.SEARCH_POINT, Routes.PLANNED_SEARCH_TO_DROPOFF, 30);
    assertRouteClearance(Routes.DROPOFF_ENTRY, Routes.PLANNED_DROPOFF_TO_SEARCH, 30);
    for (int jarIndex = 0; jarIndex < Config.JAR_COUNT; jarIndex++) {
      // Placement deliberately drives up to the pads.
      assertRouteClearance(Routes.DROPOFF_ENTRY, Routes.placementPath(jarIndex), 15);
    }
  }

  /** Documents the v1 bug fixed in Mission: the "retry" route after a failed search. */
  @Test
  void v1RetryLegFromSearchPointToDropoffCrossesStaticBoxes() {
    int crossed = 0;
    for (WorldFile.Obstacle o : world.staticObstacles()) {
      if (WorldFile.clearance(Routes.SEARCH_POINT, Routes.DROPOFF_ENTRY, o) == 0) {
        crossed++;
      }
    }
    assertEquals(4, crossed);
  }

  private static void assertRouteClearance(Vec2 start, List<Vec2> route, double minCm) {
    Vec2 here = start;
    for (Vec2 next : route) {
      for (WorldFile.Obstacle o : world.staticObstacles()) {
        double c = WorldFile.clearance(here, next, o);
        assertTrue(c >= minCm, here + " -> " + next + " passes " + c + " cm from " + o.name);
      }
      here = next;
    }
  }
}
