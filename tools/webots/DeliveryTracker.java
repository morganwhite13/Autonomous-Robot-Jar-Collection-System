import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;

/**
 * Decides when each jar has been delivered, from the jar positions alone, so v1 and v2 are
 * judged by the same rule. A jar counts as delivered once it has come to rest in the drop-off
 * lane in front of the five pads and stayed still for {@link #STILL_SECONDS}. A jar that later
 * leaves the lane (knocked away) stops counting. Positions are in the controllers' cm frame.
 */
final class DeliveryTracker {
  /** Drop-off lane: in front of the pads at x = -967, which sit at y = 602 to 722. */
  static final double ZONE_MAX_X = -930;
  static final double ZONE_MIN_Y = 590;
  static final double ZONE_MAX_Y = 750;
  /** A jar is "at rest" while it stays within this distance of where it stopped. */
  static final double STILL_CM = 1.0;
  static final double STILL_SECONDS = 5.0;

  private final double[] anchorX;
  private final double[] anchorY;
  private final double[] stillSince;
  private final double[] deliveredAt;
  private int disturbances;

  DeliveryTracker(int jarCount) {
    anchorX = new double[jarCount];
    anchorY = new double[jarCount];
    stillSince = new double[jarCount];
    deliveredAt = new double[jarCount];
    Arrays.fill(anchorX, Double.NaN);
    Arrays.fill(deliveredAt, Double.NaN);
  }

  void update(int jar, double timeS, double x, double y) {
    if (Double.isNaN(anchorX[jar]) || Math.hypot(x - anchorX[jar], y - anchorY[jar]) > STILL_CM) {
      anchorX[jar] = x;
      anchorY[jar] = y;
      stillSince[jar] = timeS;
    }
    boolean inZone = x < ZONE_MAX_X && y >= ZONE_MIN_Y && y <= ZONE_MAX_Y;
    if (!Double.isNaN(deliveredAt[jar]) && !inZone) {
      deliveredAt[jar] = Double.NaN; // knocked out of the lane after delivery
      disturbances++;
    } else if (Double.isNaN(deliveredAt[jar]) && inZone && timeS - stillSince[jar] >= STILL_SECONDS) {
      deliveredAt[jar] = stillSince[jar];
    }
  }

  int jarCount() {
    return deliveredAt.length;
  }

  int delivered() {
    int n = 0;
    for (double t : deliveredAt) {
      if (!Double.isNaN(t)) {
        n++;
      }
    }
    return n;
  }

  boolean allDelivered() {
    return delivered() == deliveredAt.length;
  }

  /** Delivery times in seconds, in delivery order. */
  List<Double> deliveryTimes() {
    List<Double> times = new ArrayList<>();
    for (double t : deliveredAt) {
      if (!Double.isNaN(t)) {
        times.add(t);
      }
    }
    times.sort(null);
    return times;
  }

  double lastDeliveryTime() {
    List<Double> times = deliveryTimes();
    return times.isEmpty() ? Double.NaN : times.get(times.size() - 1);
  }

  int disturbances() {
    return disturbances;
  }

  /** Result file contents (java.util.Properties format). */
  String toProperties(double simTimeS, boolean timeLimitHit) {
    StringBuilder times = new StringBuilder();
    for (double t : deliveryTimes()) {
      times.append(times.length() == 0 ? "" : ",").append(String.format(Locale.ROOT, "%.2f", t));
    }
    return String.format(Locale.ROOT,
        "jars=%d%ndelivered=%d%ndeliveryTimesS=%s%nsimTimeS=%.2f%ntimeLimitHit=%b%ndisturbances=%d%n",
        jarCount(), delivered(), times, simTimeS, timeLimitHit, disturbances);
  }
}
