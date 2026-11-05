import java.util.Locale;

/**
 * Immutable point in the controller's planar frame, in centimetres.
 *
 * <p>Uses the same mapping as v1: {@code x = worldX * 100}, {@code y = -worldZ * 100}
 * (the world file uses the NUE coordinate system, so world Z points "down" the map).
 */
final class Vec2 {
  final double x;
  final double y;

  Vec2(double x, double y) {
    this.x = x;
    this.y = y;
  }

  static Vec2 of(double x, double y) {
    return new Vec2(x, y);
  }

  double distanceTo(Vec2 other) {
    return Math.hypot(other.x - x, other.y - y);
  }

  /** Direction from this point to {@code other} in degrees [0, 360), i.e. atan2(dy, dx) as in v1. */
  double bearingTo(Vec2 other) {
    return Angles.normalize(Math.toDegrees(Math.atan2(other.y - y, other.x - x)));
  }

  @Override
  public boolean equals(Object o) {
    if (!(o instanceof Vec2)) {
      return false;
    }
    Vec2 v = (Vec2) o;
    return Double.compare(x, v.x) == 0 && Double.compare(y, v.y) == 0;
  }

  @Override
  public int hashCode() {
    return 31 * Double.hashCode(x) + Double.hashCode(y);
  }

  @Override
  public String toString() {
    return String.format(Locale.ROOT, "(%.0f, %.0f)", x, y);
  }
}
