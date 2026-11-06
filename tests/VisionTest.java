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

  // Exact boundaries of v1's rules (mutation testing showed these were not pinned down).

  @Test
  void steeringMarginIsStrictlyMoreThanTenPixels() {
    assertEquals(Vision.Steering.STRAIGHT, Vision.steer(new Vision.SectorCounts(20, 10, 10)));
    assertEquals(Vision.Steering.LEFT, Vision.steer(new Vision.SectorCounts(21, 10, 10)));
    assertEquals(Vision.Steering.STRAIGHT, Vision.steer(new Vision.SectorCounts(10, 10, 20)));
    assertEquals(Vision.Steering.RIGHT, Vision.steer(new Vision.SectorCounts(10, 10, 21)));
    assertEquals(Vision.Steering.UNCLEAR, Vision.steer(new Vision.SectorCounts(10, 20, 0)));
    assertEquals(Vision.Steering.STRAIGHT, Vision.steer(new Vision.SectorCounts(0, 21, 10)));
  }

  @Test
  void straightAheadNeedsMoreThanThirtyPixelsWhenNoThirdLeads() {
    assertEquals(Vision.Steering.UNCLEAR, Vision.steer(new Vision.SectorCounts(10, 10, 10)));
    assertEquals(Vision.Steering.STRAIGHT, Vision.steer(new Vision.SectorCounts(11, 10, 10)));
  }

  @Test
  void greenThresholdsAndMarginAreExact() {
    assertFalse(Vision.isJarGreen(100, 110, 50)); // green beats red by exactly 10
    assertTrue(Vision.isJarGreen(100, 111, 50));
    assertFalse(Vision.isJarGreen(50, 110, 100)); // green beats blue by exactly 10
    assertTrue(Vision.isJarGreen(50, 111, 100));
    assertTrue(Vision.isJarGreen(119, 200, 0));
    assertFalse(Vision.isJarGreen(120, 200, 0));
    assertTrue(Vision.isJarGreen(0, 36, 0));
    assertFalse(Vision.isJarGreen(0, 35, 0));
    assertFalse(Vision.isJarGreen(0, 200, 120));
  }

  @Test
  void dominantSectorNeedsToBeatBothOthers() {
    assertEquals(Vision.Sector.LEFT, new Vision.SectorCounts(20, 10, 10).dominant());
    assertEquals(Vision.Sector.RIGHT, new Vision.SectorCounts(10, 10, 20).dominant());
    assertEquals(Vision.Sector.CENTER, new Vision.SectorCounts(10, 10, 5).dominant());
    assertEquals(Vision.Sector.CENTER, new Vision.SectorCounts(5, 10, 10).dominant());
    assertEquals(Vision.Sector.CENTER, new Vision.SectorCounts(10, 5, 10).dominant());
    assertEquals("[L:1 C:2 R:3]", new Vision.SectorCounts(1, 2, 3).toString());
  }

  @Test
  void eachHalfOfEverySteeringRuleHasItsOwnBoundary() {
    // left must beat centre AND right by more than 10; check each comparison on its own
    assertEquals(Vision.Steering.UNCLEAR, Vision.steer(new Vision.SectorCounts(20, 10, 0)));
    assertEquals(Vision.Steering.UNCLEAR, Vision.steer(new Vision.SectorCounts(20, 0, 10)));
    assertEquals(Vision.Steering.LEFT, Vision.steer(new Vision.SectorCounts(21, 10, 0)));
    assertEquals(Vision.Steering.LEFT, Vision.steer(new Vision.SectorCounts(21, 0, 10)));
    assertEquals(Vision.Steering.UNCLEAR, Vision.steer(new Vision.SectorCounts(10, 0, 20)));
    assertEquals(Vision.Steering.UNCLEAR, Vision.steer(new Vision.SectorCounts(0, 10, 20)));
    assertEquals(Vision.Steering.RIGHT, Vision.steer(new Vision.SectorCounts(10, 0, 21)));
    assertEquals(Vision.Steering.RIGHT, Vision.steer(new Vision.SectorCounts(0, 10, 21)));
    assertEquals(Vision.Steering.UNCLEAR, Vision.steer(new Vision.SectorCounts(0, 20, 10)));
    assertEquals(Vision.Steering.STRAIGHT, Vision.steer(new Vision.SectorCounts(0, 21, 10)));
    assertEquals(Vision.Steering.STRAIGHT, Vision.steer(new Vision.SectorCounts(10, 21, 0)));
  }

  /** A 64x64 frame with green filling the given rectangle (inclusive). */
  private static RgbImage frameWith(int[]... rects) {
    int[] px = new int[64 * 64];
    java.util.Arrays.fill(px, RgbImage.pack(190, 150, 110));
    for (int[] r : rects) {
      for (int y = r[1]; y <= r[3]; y++) {
        for (int x = r[0]; x <= r[2]; x++) {
          px[y * 64 + x] = RgbImage.pack(30, 160, 30);
        }
      }
    }
    return new RgbImage(64, 64, px);
  }

  @Test
  void blobsSplitJarsThatDontTouch() {
    java.util.List<Vision.Blob> blobs = Vision.blobs(frameWith(new int[] {5, 40, 10, 50},
        new int[] {30, 38, 33, 44}));
    assertEquals(2, blobs.size());
    assertEquals(5, blobs.get(0).fromColumn);
    assertEquals(10, blobs.get(0).toColumn);
    assertEquals(50, blobs.get(0).lowestRow);
    assertEquals(6 * 11, blobs.get(0).pixels);
    assertEquals(31.5, blobs.get(1).centreColumn(), 1e-9);
    assertEquals(50, Vision.lowestGreenRow(frameWith(new int[] {5, 40, 10, 50}, new int[] {30, 38, 33, 44})));
    assertEquals(-1, Vision.lowestGreenRow(frameWith()));
  }

  @Test
  void distanceFromTheLowestRowMatchesTheWebotsCalibration() {
    // Jar straight ahead in Webots: robot-to-jar distance and the lowest green row seen.
    int[][] measured = {{64, 60}, {74, 55}, {84, 51}, {94, 49}, {104, 47}, {114, 45}, {124, 44}};
    for (int[] m : measured) {
      assertEquals(m[0], Vision.distanceCm(m[1]), 7, "row " + m[1]);
    }
    assertEquals(Double.POSITIVE_INFINITY, Vision.distanceCm(31));
    assertEquals(Double.POSITIVE_INFINITY, Vision.distanceCm(-1));
  }

  @Test
  void bearingOffsetIsPositiveToTheLeftAndZeroInTheMiddle() {
    assertEquals(0, Vision.bearingOffsetDeg(31.5), 1e-9);
    assertTrue(Vision.bearingOffsetDeg(0) > 21 && Vision.bearingOffsetDeg(0) < 23);
    assertEquals(-Vision.bearingOffsetDeg(0), Vision.bearingOffsetDeg(63), 1e-9);
  }

  @Test
  void leanPointsAtTheOuterThirdWithMoreGreen() {
    assertEquals(Vision.Steering.RIGHT, Vision.lean(new Vision.SectorCounts(0, 5, 15)));
    assertEquals(Vision.Steering.RIGHT, Vision.lean(new Vision.SectorCounts(4, 9, 5)));
    assertEquals(Vision.Steering.LEFT, Vision.lean(new Vision.SectorCounts(15, 5, 0)));
    assertEquals(Vision.Steering.LEFT, Vision.lean(new Vision.SectorCounts(5, 20, 4)));
    assertEquals(Vision.Steering.STRAIGHT, Vision.lean(new Vision.SectorCounts(5, 5, 5)));
    assertEquals(Vision.Steering.STRAIGHT, Vision.lean(new Vision.SectorCounts(0, 20, 0)));
  }
}
