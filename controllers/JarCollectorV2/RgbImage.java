/**
 * A camera frame decoupled from the Webots API so the vision code can be unit-tested.
 * Pixels are stored packed as {@code 0xRRGGBB}, row-major.
 */
final class RgbImage {
  final int width;
  final int height;
  private final int[] rgb;

  RgbImage(int width, int height, int[] rgb) {
    if (width <= 0 || height <= 0 || rgb.length != width * height) {
      throw new IllegalArgumentException(
          "expected " + width + "x" + height + " pixels, got " + rgb.length);
    }
    this.width = width;
    this.height = height;
    this.rgb = rgb.clone();
  }

  static int pack(int r, int g, int b) {
    return ((r & 0xFF) << 16) | ((g & 0xFF) << 8) | (b & 0xFF);
  }

  int red(int x, int y) {
    return (rgb[y * width + x] >> 16) & 0xFF;
  }

  int green(int x, int y) {
    return (rgb[y * width + x] >> 8) & 0xFF;
  }

  int blue(int x, int y) {
    return rgb[y * width + x] & 0xFF;
  }
}
