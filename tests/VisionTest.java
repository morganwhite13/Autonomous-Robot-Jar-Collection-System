import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Arrays;
import java.util.Random;
import org.junit.jupiter.api.Test;

class VisionTest {

  private static RgbImage filled(int r, int g, int b) {
    int[] px = new int[64 * 64];
    Arrays.fill(px, RgbImage.pack(r, g, b));
    return new RgbImage(64, 64, px);
  }

  @Test
  void acceptsJarGreenIncludingShadedAndWashedOut() {
    assertTrue(Vision.isJarGreen(30, 160, 30));
    assertTrue(Vision.isJarGreen(10, 60, 10)); // jar in shadow
    assertTrue(Vision.isJarGreen(100, 130, 115)); // desaturated by lighting
  }

  @Test
  void rejectsGreyAndBrownThatV1CountedAsGreen() {
    int[][] notGreen = {{50, 50, 50}, {100, 80, 60}, {90, 90, 100}, {110, 100, 40}};
    for (int[] c : notGreen) {
      assertTrue(LegacyV1.isGreen(c[0], c[1], c[2]), "v1 accepted " + Arrays.toString(c));
      assertFalse(Vision.isJarGreen(c[0], c[1], c[2]), "v2 rejects " + Arrays.toString(c));
    }
  }

  @Test
  void v1CompatibleMarginReproducesV1Exactly() {
    for (int r = 0; r < 256; r += 3) {
      for (int g = 0; g < 256; g += 3) {
        for (int b = 0; b < 256; b += 3) {
          assertEquals(LegacyV1.isGreen(r, g, b),
              Vision.isJarGreen(r, g, b, Vision.V1_COMPATIBLE_MARGIN), r + "," + g + "," + b);
        }
      }
    }
  }

  @Test
  void v2OnlyEverRemovesPixelsFromV1sSet() {
    for (int r = 0; r < 256; r += 3) {
      for (int g = 0; g < 256; g += 3) {
        for (int b = 0; b < 256; b += 3) {
          if (Vision.isJarGreen(r, g, b)) {
            assertTrue(LegacyV1.isGreen(r, g, b));
          }
        }
      }
    }
  }

  @Test
  void sectorCountsMatchV1CountColorOnRandomFrames() {
    Random random = new Random(7);
    for (int frame = 0; frame < 200; frame++) {
      int[] px = new int[64 * 64];
      for (int i = 0; i < px.length; i++) {
        px[i] = RgbImage.pack(random.nextInt(256), random.nextInt(256), random.nextInt(256));
      }
      RgbImage image = new RgbImage(64, 64, px);
      Vision.SectorCounts v2 = Vision.countGreen(image, Vision.V1_COMPATIBLE_MARGIN);
      int[][] v1 = LegacyV1.countColor(image);
      assertArrayEquals(v1[0], new int[] {v2.left, v2.center, v2.right});
    }
  }

  @Test
  void frameIsSplit21_22_21LikeV1() {
    Vision.SectorCounts c = Vision.countGreen(filled(20, 200, 20));
    assertEquals(21 * 64, c.left);
    assertEquals(22 * 64, c.center);
    assertEquals(21 * 64, c.right);
  }

  @Test
  void steeringFollowsV1Rules() {
    assertEquals(Vision.Steering.LEFT, Vision.steer(new Vision.SectorCounts(40, 10, 10)));
    assertEquals(Vision.Steering.RIGHT, Vision.steer(new Vision.SectorCounts(10, 10, 40)));
    assertEquals(Vision.Steering.STRAIGHT, Vision.steer(new Vision.SectorCounts(10, 40, 10)));
    assertEquals(Vision.Steering.STRAIGHT, Vision.steer(new Vision.SectorCounts(15, 12, 14)));
    assertEquals(Vision.Steering.UNCLEAR, Vision.steer(new Vision.SectorCounts(5, 6, 4)));
  }

  @Test
  void scanScoreGivesCentredJarsABonus() {
    assertEquals(Vision.Sector.CENTER, new Vision.SectorCounts(10, 5, 10).dominant());
    assertEquals(30 + Config.SCAN_CENTER_BONUS, Vision.scanScore(new Vision.SectorCounts(5, 20, 5)));
    assertEquals(30, Vision.scanScore(new Vision.SectorCounts(20, 5, 5)));
  }
}
