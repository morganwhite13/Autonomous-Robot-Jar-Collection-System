import java.util.List;
import java.util.Locale;

/**
 * Waypoint navigation: turn in place towards a waypoint, then drive to it.
 *
 * <p>Same strategy as v1's makeHardcodedTurn / moveHardcodedAhead, with these fixes:
 * <ul>
 *   <li>Turns use a seam-safe heading error. v1 tested {@code final-3 < h < final+3} on raw
 *       values, so a target near 0/360 had a narrower (or split) acceptance window.</li>
 *   <li>Turns slow down near the target instead of switching between full-speed left and right,
 *       and stop immediately if already aligned (v1 always rotated for one step first).</li>
 *   <li>Driving steers towards the waypoint. v1 drove open-loop, so a 3 degree heading error
 *       became about 28 cm of sideways error over the longest (530 cm) leg.</li>
 *   <li>Drives detect stalls and have a timeout. v1 looped forever if the robot was blocked.</li>
 * </ul>
 */
final class Navigator {

  enum DriveResult { ARRIVED, STALLED, TIMED_OUT }

  private final RobotPlatform robot;
  private final MissionStats stats;

  Navigator(RobotPlatform robot, MissionStats stats) {
    this.robot = robot;
    this.stats = stats;
  }

  /** Visits every waypoint in order. Returns false if any leg did not arrive cleanly. */
  boolean followRoute(List<Vec2> waypoints) {
    boolean clean = true;
    for (Vec2 waypoint : waypoints) {
      clean &= goTo(waypoint) == DriveResult.ARRIVED;
    }
    return clean;
  }

  DriveResult goTo(Vec2 target) {
    Vec2 here = robot.positionCm();
    if (here.distanceTo(target) < 1.0) {
      return DriveResult.ARRIVED;
    }
    turnTo(here.bearingTo(target), Config.MAX_SPEED);
    return driveTo(target);
  }

  /**
   * Rotates in place to {@code targetDeg}. Full speed while the error is large, proportional
   * slow-down inside {@link Config#TURN_SLOWDOWN_DEG}. Returns false on timeout.
   */
  boolean turnTo(double targetDeg, double maxSpeed) {
    double minSpeed = Math.min(Config.TURN_MIN_SPEED, maxSpeed);
    for (int i = 0; i < Config.TURN_TIMEOUT_STEPS; i++) {
      double error = Angles.delta(robot.headingDeg(), targetDeg);
      if (Math.abs(error) <= Config.HEADING_TOLERANCE_DEG) {
        robot.stop();
        return true;
      }
      double speed = clamp(maxSpeed * Math.abs(error) / Config.TURN_SLOWDOWN_DEG, minSpeed, maxSpeed);
      if (error > 0) {
        robot.drive(-speed, speed); // counter-clockwise (left)
      } else {
        robot.drive(speed, -speed); // clockwise (right)
      }
      robot.step();
    }
    robot.stop();
    stats.turnTimeouts++;
    Log.warn(String.format(Locale.ROOT, "turn to %.0f deg timed out at %.0f deg",
        targetDeg, robot.headingDeg()));
    return false;
  }

  /**
   * Drives forward until the robot is as far from its start point as the target was (v1's
   * stopping rule), steering towards the target on the way.
   */
  DriveResult driveTo(Vec2 target) {
    Vec2 start = robot.positionCm();
    double length = start.distanceTo(target);
    int timeout = Math.max(Config.DRIVE_TIMEOUT_MIN_STEPS,
        (int) Math.ceil(length * Config.DRIVE_TIMEOUT_STEPS_PER_CM));

    Vec2 checkpoint = start;
    int stepsSinceCheckpoint = 0;
    for (int i = 0; i < timeout; i++) {
      Vec2 here = robot.positionCm();
      if (here.distanceTo(start) >= length) {
        robot.stop();
        return DriveResult.ARRIVED;
      }

      double correction = 0;
      if (here.distanceTo(target) > Config.HEADING_HOLD_MIN_DISTANCE_CM) {
        double error = Angles.delta(robot.headingDeg(), here.bearingTo(target));
        correction = clamp(error * Config.HEADING_HOLD_GAIN,
            -Config.HEADING_HOLD_MAX_CORRECTION, Config.HEADING_HOLD_MAX_CORRECTION);
      }
      // Positive error = target is to the left, so speed up the right wheel.
      robot.drive(
          clamp(Config.MAX_SPEED - correction, -Config.MAX_SPEED, Config.MAX_SPEED),
          clamp(Config.MAX_SPEED + correction, -Config.MAX_SPEED, Config.MAX_SPEED));
      robot.step();

      if (++stepsSinceCheckpoint >= Config.STALL_WINDOW_STEPS) {
        Vec2 now = robot.positionCm();
        if (now.distanceTo(checkpoint) < Config.STALL_MIN_PROGRESS_CM) {
          robot.stop();
          stats.driveStalls++;
          Log.warn("stalled at " + now + " while driving to " + target + "; skipping to next leg");
          return DriveResult.STALLED;
        }
        checkpoint = now;
        stepsSinceCheckpoint = 0;
      }
    }
    robot.stop();
    stats.driveTimeouts++;
    Log.warn("drive to " + target + " timed out at " + robot.positionCm());
    return DriveResult.TIMED_OUT;
  }

  /** Open-loop straight drive for a fixed number of steps (v1 moveRandomly / backUpDistance). */
  void driveSteps(double speed, int steps) {
    robot.drive(speed, speed);
    robot.waitSteps(steps);
    robot.stop();
  }

  static double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }
}
