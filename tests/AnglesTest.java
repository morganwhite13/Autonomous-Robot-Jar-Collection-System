import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;
import org.junit.jupiter.api.Test;

class AnglesTest {

  @Test
  void normalizeMapsIntoZeroTo360() {
    assertEquals(0, Angles.normalize(360), 1e-9);
    assertEquals(350, Angles.normalize(-10), 1e-9);
    assertEquals(10, Angles.normalize(730), 1e-9);
    assertTrue(Angles.normalize(-1e-15) < 360, "rounding must not produce 360");
  }

  @Test
  void deltaTakesTheShortWayAcrossTheSeam() {
    assertEquals(10, Angles.delta(355, 5), 1e-9);
    assertEquals(-10, Angles.delta(5, 355), 1e-9);
    assertEquals(-90, Angles.delta(90, 0), 1e-9);
    assertEquals(180, Angles.delta(0, 180), 1e-9);
  }

  @Test
  void withinIsSeamSafe() {
    // v1's check "final - 3 < h < final + 3" fails for these.
    assertTrue(Angles.within(359, 1, 3));
    assertTrue(Angles.within(1, 359, 3));
    assertFalse(Angles.within(10, 20, 3));
  }

  @Test
  void compassConversionMatchesV1ExceptForTruncation() {
    Random random = new Random(1);
    for (int i = 0; i < 20_000; i++) {
      double[] v = {random.nextGaussian(), random.nextGaussian(), random.nextGaussian()};
      int v1 = LegacyV1.compassDegrees(v);
      double v2 = Angles.fromCompass(v);
      assertTrue(Angles.within(v2, v1, 1.0), "v1=" + v1 + " v2=" + v2);
      assertEquals(v1, LegacyV1.compassDegrees(v2), "heading mapping used by the v1 ports");
    }
  }

  @Test
  void bearingUsesAtan2LikeV1() {
    assertEquals(90, Vec2.of(0, 0).bearingTo(Vec2.of(0, 10)), 1e-9);
    assertEquals(270, Routes.TO_SEARCH_POINT.get(0).bearingTo(Routes.SEARCH_POINT), 1e-9);
  }
}
