import java.util.ArrayList;
import java.util.List;

/**
 * Jar detection from a camera frame. Pure functions, no Webots dependency.
 *
 * <p>The frame is split into three vertical sectors exactly as in v1: columns [0, w/3),
 * [w/3, w - w/3) and [w - w/3, w). For the 64-pixel camera that is 21 / 22 / 21 columns.
 */
final class Vision {
  private Vision() {}

  /** Dominance margin that makes {@link #isJarGreen(int, int, int, int)} identical to v1. */
  static final int V1_COMPATIBLE_MARGIN = -255;

  enum Sector { LEFT, CENTER, RIGHT }

  enum Steering { LEFT, RIGHT, STRAIGHT, UNCLEAR }

  /** Number of jar-green pixels in each third of the frame. */
  static final class SectorCounts {
    final int left;
    final int center;
    final int right;

    SectorCounts(int left, int center, int right) {
      this.left = left;
      this.center = center;
      this.right = right;
    }

    int total() {
      return left + center + right;
    }

    /** v1 rule: a side wins only if it beats both others; ties fall back to CENTER. */
    Sector dominant() {
      if (left > center && left > right) {
        return Sector.LEFT;
      }
      if (right > center && right > left) {
        return Sector.RIGHT;
      }
      return Sector.CENTER;
    }

    @Override
    public String toString() {
      return "[L:" + left + " C:" + center + " R:" + right + "]";
    }
  }

  static boolean isJarGreen(int r, int g, int b) {
    return isJarGreen(r, g, b, Config.GREEN_DOMINANCE_MARGIN);
  }

  /**
   * v1's absolute thresholds, plus (new) a requirement that green is the dominant channel.
   * With {@code dominanceMargin = V1_COMPATIBLE_MARGIN} this is exactly v1's test.
   */
  static boolean isJarGreen(int r, int g, int b, int dominanceMargin) {
    boolean v1Thresholds =
        r < Config.GREEN_MAX_RED && g > Config.GREEN_MIN_GREEN && b < Config.GREEN_MAX_BLUE;
    return v1Thresholds && g > r + dominanceMargin && g > b + dominanceMargin;
  }

  static SectorCounts countGreen(RgbImage image) {
    return countGreen(image, Config.GREEN_DOMINANCE_MARGIN);
  }

  static SectorCounts countGreen(RgbImage image, int dominanceMargin) {
    int third = image.width / 3;
    int rightStart = image.width - third;
    int left = 0;
    int center = 0;
    int right = 0;
    for (int y = 0; y < image.height; y++) {
      for (int x = 0; x < image.width; x++) {
        if (!isJarGreen(image.red(x, y), image.green(x, y), image.blue(x, y), dominanceMargin)) {
          continue;
        }
        if (x < third) {
          left++;
        } else if (x < rightStart) {
          center++;
        } else {
          right++;
        }
      }
    }
    return new SectorCounts(left, center, right);
  }

  /** v1 approach steering, unchanged: steer towards a sector that clearly leads the others. */
  static Steering steer(SectorCounts c) {
    int m = Config.STEER_MARGIN_PIXELS;
    boolean left = c.left > c.center + m && c.left > c.right + m;
    boolean center = c.center > c.left + m && c.center > c.right + m;
    boolean right = c.right > c.left + m && c.right > c.center + m;
    if (left) {
      return Steering.LEFT;
    }
    if (right) {
      return Steering.RIGHT;
    }
    if (center || c.total() > Config.STEER_STRAIGHT_MIN_PIXELS) {
      return Steering.STRAIGHT;
    }
    return Steering.UNCLEAR;
  }

  /**
   * new ({@link Config#STEER_WHEN_UNCLEAR}): which way the green leans when {@link #steer} can't
   * decide: towards the outer third with more of it, or straight if they're level.
   */
  static Steering lean(SectorCounts c) {
    if (c.right > c.left) {
      return Steering.RIGHT;
    }
    if (c.left > c.right) {
      return Steering.LEFT;
    }
    return Steering.STRAIGHT;
  }

  /** v1 scan score: number of green pixels, +50 when the jar is in the centre third. */
  static int scanScore(SectorCounts c) {
    return c.total() + (c.dominant() == Sector.CENTER ? Config.SCAN_CENTER_BONUS : 0);
  }

  /** A run of adjacent image columns that contain jar-green pixels: usually one jar. */
  static final class Blob {
    final int fromColumn;
    final int toColumn;
    final int lowestRow;
    final int pixels;

    Blob(int fromColumn, int toColumn, int lowestRow, int pixels) {
      this.fromColumn = fromColumn;
      this.toColumn = toColumn;
      this.lowestRow = lowestRow;
      this.pixels = pixels;
    }

    double centreColumn() {
      return (fromColumn + toColumn) / 2.0;
    }
  }

  /** The jar-green blobs in a frame, left to right. */
  static List<Blob> blobs(RgbImage image) {
    List<Blob> blobs = new ArrayList<>();
    int start = -1;
    int lowest = -1;
    int pixels = 0;
    for (int x = 0; x <= image.width; x++) {
      int columnPixels = 0;
      int columnLowest = -1;
      for (int y = 0; x < image.width && y < image.height; y++) {
        if (isJarGreen(image.red(x, y), image.green(x, y), image.blue(x, y))) {
          columnPixels++;
          columnLowest = y;
        }
      }
      if (columnPixels > 0) {
        if (start < 0) {
          start = x;
        }
        lowest = Math.max(lowest, columnLowest);
        pixels += columnPixels;
      } else if (start >= 0) {
        blobs.add(new Blob(start, x - 1, lowest, pixels));
        start = -1;
        lowest = -1;
        pixels = 0;
      }
    }
    return blobs;
  }

  /** Lowest image row (largest index) holding a jar-green pixel, or -1 if there is none. */
  static int lowestGreenRow(RgbImage image) {
    int lowest = -1;
    for (Blob b : blobs(image)) {
      lowest = Math.max(lowest, b.lowestRow);
    }
    return lowest;
  }

  /**
   * Distance from the robot's centre to the centre of a jar whose lowest pixel is in
   * {@code row}, from the camera geometry in {@link Config}. Infinite if the row is at or above
   * the horizon (or there is no jar).
   */
  static double distanceCm(int row) {
    double belowHorizon = row - (IMAGE_ROWS - 1) / 2.0;
    if (row < 0 || belowHorizon <= 0) {
      return Double.POSITIVE_INFINITY;
    }
    return Config.CAMERA_HEIGHT_CM * Config.CAMERA_FOCAL_PX / belowHorizon
        + Config.CAMERA_AHEAD_CM + Config.JAR_RADIUS_CM;
  }

  /** How far left of the camera's axis (degrees) something in this column is. */
  static double bearingOffsetDeg(double column) {
    return Math.toDegrees(Math.atan(((IMAGE_COLUMNS - 1) / 2.0 - column) / Config.CAMERA_FOCAL_PX));
  }

  /** The camera's resolution (Webots default, 64 x 64). */
  static final int IMAGE_ROWS = 64;
  static final int IMAGE_COLUMNS = 64;
}
