/**
 * Heading arithmetic. All angles are in degrees.
 *
 * <p>Convention (inherited from v1): headings increase counter-clockwise, so a positive
 * {@link #delta} means "turn left" (left wheel backwards, right wheel forwards).
 */
final class Angles {
  private Angles() {}

  /** Maps any angle onto [0, 360). */
  static double normalize(double deg) {
    double d = deg % 360.0;
    if (d < 0) {
      d += 360.0;
    }
    return d >= 360.0 ? d - 360.0 : d; // guards the -1e-15 + 360 == 360.0 rounding case
  }

  /**
   * Signed shortest rotation from heading {@code from} to heading {@code to}, in (-180, 180].
   * Positive means a counter-clockwise (left) turn. Correct across the 0/360 seam.
   */
  static double delta(double from, double to) {
    double d = normalize(to - from);
    return d > 180.0 ? d - 360.0 : d;
  }

  /** True if the two headings are within {@code tolerance} degrees of each other (seam-safe). */
  static boolean within(double a, double b, double tolerance) {
    return Math.abs(delta(a, b)) <= tolerance;
  }

  /**
   * Converts raw compass values to a heading in [0, 360).
   *
   * <p>Same formula as v1's {@code getCompassReadingInDegrees}, but without truncating to an
   * int (v1 lost up to 1 degree there) and normalised once here instead of at every call site.
   */
  static double fromCompass(double[] values) {
    double rad = Math.atan2(values[0], values[2]);
    return normalize(-Math.toDegrees(rad + Math.PI));
  }
}
