import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import org.junit.jupiter.api.Test;

/** RgbImage and Vec2. The vision tests read pixels through RgbImage on both sides of their
 *  v1 comparisons, so channel decoding needs its own direct checks. */
class ValueTypesTest {

  @Test
  void packedPixelsDecodeToTheirChannels() {
    int[] px = {RgbImage.pack(255, 0, 128), RgbImage.pack(1, 2, 3), RgbImage.pack(0, 255, 0), 0};
    RgbImage image = new RgbImage(2, 2, px);
    assertEquals(255, image.red(0, 0));
    assertEquals(0, image.green(0, 0));
    assertEquals(128, image.blue(0, 0));
    assertEquals(1, image.red(1, 0));
    assertEquals(2, image.green(1, 0));
    assertEquals(3, image.blue(1, 0));
    assertEquals(255, image.green(0, 1));
    assertEquals(0, image.red(1, 1) + image.green(1, 1) + image.blue(1, 1));
  }

  @Test
  void imageSizeMustMatchItsPixels() {
    assertThrows(IllegalArgumentException.class, () -> new RgbImage(0, 1, new int[0]));
    assertThrows(IllegalArgumentException.class, () -> new RgbImage(1, 0, new int[0]));
    assertThrows(IllegalArgumentException.class, () -> new RgbImage(2, 2, new int[3]));
    assertEquals(1, new RgbImage(1, 1, new int[1]).width);
  }

  @Test
  void imageCopiesItsPixels() {
    int[] px = {RgbImage.pack(10, 20, 30)};
    RgbImage image = new RgbImage(1, 1, px);
    px[0] = 0;
    assertEquals(20, image.green(0, 0));
  }

  @Test
  void vec2ValueSemantics() {
    assertEquals(Vec2.of(1, 2), Vec2.of(1, 2));
    assertEquals(Vec2.of(1, 2).hashCode(), Vec2.of(1, 2).hashCode());
    assertNotEquals(Vec2.of(1, 2), Vec2.of(2, 1));
    assertNotEquals(Vec2.of(1, 2), "(1, 2)");
    assertEquals("(1, 2)", Vec2.of(1.4, 2.4).toString());
    assertEquals(5, Vec2.of(0, 0).distanceTo(Vec2.of(3, 4)), 1e-12);
  }

  @Test
  void differentPointsUsuallyHashDifferently() {
    assertNotEquals(Vec2.of(1, 2).hashCode(), Vec2.of(2, 1).hashCode());
    assertNotEquals(Vec2.of(0, 0).hashCode(), Vec2.of(0, 1).hashCode());
  }
}
