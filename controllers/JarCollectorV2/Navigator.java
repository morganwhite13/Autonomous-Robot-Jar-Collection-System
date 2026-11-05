import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.function.BooleanSupplier;

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
 *   <li>Optionally ({@link Config#SONAR_WATCH}), travel legs use the front sonars to slow down for
 *       and stop short of obstacles. v1 never used the sonars.</li>
 *   <li>Routes driven with a jar in the gripper turn more slowly
 *       ({@link Config#CARRY_TURN_SPEED}). v1 turned at full speed, which throws the jar out.</li>
 *   <li>A route can be abandoned as soon as a condition stops holding (the jar is no longer in
 *       the gripper), and a stalled leg can plan a way around what stopped it
 *       ({@link Config#DETOUR_AROUND_STALLS}).</li>
 * </ul>
 */
final class Navigator {

  enum DriveResult { ARRIVED, STALLED, TIMED_OUT, OBSTACLE_AHEAD, ABORTED }

  enum RouteResult { CLEAN, WITH_PROBLEMS, ABORTED }

  private static final BooleanSupplier ALWAYS = () -> true;

  /** How to drive a route. The defaults are v2's plain behaviour. */
  static final class Travel {
    private boolean watchAhead;
    private double turnSpeed = Config.MAX_SPEED;
    private boolean detours;
    private BooleanSupplier keepGoing = ALWAYS;

    /** Slow down for and stop short of what the front sonars see ({@link Config#SONAR_WATCH}). */
    Travel watchAhead(boolean on) {
      watchAhead = on;
      return this;
    }

    Travel turnSpeed(double radPerS) {
      turnSpeed = radPerS;
      return this;
    }

    /** Plan a way around a stalled leg ({@link Config#DETOUR_AROUND_STALLS}). */
    Travel detours(boolean on) {
      detours = on;
      return this;
    }

    /** Checked every step: once it returns false, the route is abandoned. */
    Travel keepGoingWhile(BooleanSupplier condition) {
      keepGoing = condition;
      return this;
    }
  }

  private final RobotPlatform robot;
  private final MissionStats stats;

  Navigator(RobotPlatform robot, MissionStats stats) {
    this.robot = robot;
    this.stats = stats;
  }

  /** Visits every waypoint in order. Returns false if any leg did not arrive cleanly. */
  boolean followRoute(List<Vec2> waypoints) {
    return followRoute(waypoints, false);
  }

  /**
   * Visits every waypoint in order; with {@code watchAhead}, each leg slows down for and stops
   * short of obstacles the front sonars see (see {@link Config#SONAR_WATCH}).
   */
  boolean followRoute(List<Vec2> waypoints, boolean watchAhead) {
    return followRoute(waypoints, watchAhead, Config.MAX_SPEED);
  }

  /** As {@link #followRoute(List, boolean)}, turning towards each waypoint at most this fast. */
  boolean followRoute(List<Vec2> waypoints, boolean watchAhead, double turnSpeed) {
    return drive(waypoints, new Travel().watchAhead(watchAhead).turnSpeed(turnSpeed))
        == RouteResult.CLEAN;
  }

  /** Visits every waypoint in order, as {@code travel} says. */
  RouteResult drive(List<Vec2> waypoints, Travel travel) {
    boolean clean = true;
    for (Vec2 waypoint : waypoints) {
      DriveResult result = goTo(waypoint, travel);
      if (result == DriveResult.STALLED && travel.detours && Config.DETOUR_AROUND_STALLS) {
        result = detour(waypoint, travel);
      }
      if (result == DriveResult.ABORTED) {
        return RouteResult.ABORTED;
      }
      clean &= result == DriveResult.ARRIVED;
    }
    return clean ? RouteResult.CLEAN : RouteResult.WITH_PROBLEMS;
  }

  /**
   * Drives to {@code target} along a route planned on {@link ArenaMap} from wherever the robot
   * is, for when it isn't on one of the hand-tuned routes (after dropping a jar). Drives
   * straight there if no route is found.
   */
  RouteResult driveViaPlan(Vec2 target, Travel travel) {
    List<Vec2> legs = plannedLegs(ArenaMap.planner(List.of()), target);
    if (legs.isEmpty()) {
      legs = List.of(target);
    } else {
      Log.info("Planned a route to " + target + " (" + legs.size() + " legs)");
    }
    return drive(legs, travel);
  }

  /**
   * The robot stalled on its way to {@code target}. Assume a {@link
   * Config#DETOUR_BLOCKER_SIZE_CM} square just ahead of it is in the way, back off, plan around
   * it and the static obstacles, and follow that. STALLED if there's no way round.
   */
  private DriveResult detour(Vec2 target, Travel travel) {
    Vec2 here = robot.positionCm();
    double h = Math.toRadians(robot.headingDeg());
    Vec2 ahead = Vec2.of(here.x + Config.DETOUR_BLOCKER_AHEAD_CM * Math.cos(h),
        here.y + Config.DETOUR_BLOCKER_AHEAD_CM * Math.sin(h));
    double half = Config.DETOUR_BLOCKER_SIZE_CM / 2;
    Vec2[] blocker = {Vec2.of(ahead.x - half, ahead.y - half), Vec2.of(ahead.x + half, ahead.y - half),
        Vec2.of(ahead.x + half, ahead.y + half), Vec2.of(ahead.x - half, ahead.y + half)};
    driveSteps(-Config.MANOEUVRE_SPEED, Config.DETOUR_BACKUP_STEPS);
    List<Vec2> legs = plannedLegs(ArenaMap.planner(List.<Vec2[]>of(blocker)), target);
    if (legs.isEmpty()) {
      Log.warn("no way around the obstacle at " + ahead + "; skipping to the next leg");
      return DriveResult.STALLED;
    }
    stats.recordDetour();
    Log.info("Detour around an obstacle at " + ahead + ": " + legs.size() + " legs to " + target);
    for (Vec2 leg : legs) {
      DriveResult result = goTo(leg, travel);
      if (result != DriveResult.ARRIVED) {
        return result;
      }
    }
    return DriveResult.ARRIVED;
  }

  /**
   * Legs from the robot's position to {@code target} on {@code planner}, starting and ending in
   * open floor near them if they are too close to something; empty if there's no route.
   */
  private List<Vec2> plannedLegs(RoutePlanner planner, Vec2 target) {
    Vec2 here = robot.positionCm();
    Vec2 from = planner.nearestFree(here, Config.DETOUR_FIND_FREE_CM);
    Vec2 to = planner.nearestFree(target, Config.DETOUR_FIND_FREE_CM);
    if (from == null || to == null) {
      return List.of();
    }
    List<Vec2> path = planner.plan(from, to);
    if (path.isEmpty() && !from.equals(to)) {
      return List.of();
    }
    List<Vec2> legs = new ArrayList<>();
    if (!from.equals(here)) {
      legs.add(from);
    }
    legs.addAll(path);
    if (!to.equals(target)) {
      legs.add(target);
    }
    return legs;
  }

  DriveResult goTo(Vec2 target) {
    return goTo(target, false);
  }

  DriveResult goTo(Vec2 target, boolean watchAhead) {
    return goTo(target, watchAhead, Config.MAX_SPEED);
  }

  DriveResult goTo(Vec2 target, boolean watchAhead, double turnSpeed) {
    return goTo(target, new Travel().watchAhead(watchAhead).turnSpeed(turnSpeed));
  }

  DriveResult goTo(Vec2 target, Travel travel) {
    Vec2 here = robot.positionCm();
    if (here.distanceTo(target) < 1.0) {
      return DriveResult.ARRIVED;
    }
    if (!turnTo(here.bearingTo(target), travel.turnSpeed, travel.keepGoing)
        && !travel.keepGoing.getAsBoolean()) {
      return DriveResult.ABORTED;
    }
    return driveTo(target, travel.watchAhead, travel.keepGoing);
  }

  /**
   * Rotates in place to {@code targetDeg}. Full speed while the error is large, proportional
   * slow-down inside {@link Config#TURN_SLOWDOWN_DEG}. Returns false on timeout.
   */
  boolean turnTo(double targetDeg, double maxSpeed) {
    return turnTo(targetDeg, maxSpeed, ALWAYS);
  }

  /** As {@link #turnTo(double, double)}; stops and returns false as soon as keepGoing fails. */
  private boolean turnTo(double targetDeg, double maxSpeed, BooleanSupplier keepGoing) {
    double minSpeed = Math.min(Config.TURN_MIN_SPEED, maxSpeed);
    for (int i = 0; i < Config.TURN_TIMEOUT_STEPS; i++) {
      if (!keepGoing.getAsBoolean()) {
        robot.stop();
        return false;
      }
      double error = Angles.delta(robot.headingDeg(), targetDeg);
      if (Math.abs(error) <= Config.HEADING_TOLERANCE_DEG) {
        robot.stop();
        return true;
      }
      double speed =
          clamp(maxSpeed * Math.abs(error) / Config.TURN_SLOWDOWN_DEG, minSpeed, maxSpeed);
      if (error > 0) {
        robot.drive(-speed, speed); // counter-clockwise (left)
      } else {
        robot.drive(speed, -speed); // clockwise (right)
      }
      robot.step();
    }
    robot.stop();
    stats.recordTurnTimeout();
    Log.warn(String.format(Locale.ROOT, "turn to %.0f deg timed out at %.0f deg",
        targetDeg, robot.headingDeg()));
    return false;
  }

  /**
   * Drives forward until the robot is as far from its start point as the target was (v1's
   * stopping rule), steering towards the target on the way.
   */
  DriveResult driveTo(Vec2 target) {
    return driveTo(target, false);
  }

  /** As {@link #driveTo(Vec2)}; with {@code watchAhead}, also slows down and stops for echoes. */
  DriveResult driveTo(Vec2 target, boolean watchAhead) {
    return driveTo(target, watchAhead, ALWAYS);
  }

  private DriveResult driveTo(Vec2 target, boolean watchAhead, BooleanSupplier keepGoing) {
    Vec2 start = robot.positionCm();
    double length = start.distanceTo(target);
    int timeout = Math.max(Config.DRIVE_TIMEOUT_MIN_STEPS,
        (int) Math.ceil(length * Config.DRIVE_TIMEOUT_STEPS_PER_CM));

    Vec2 checkpoint = start;
    int stepsSinceCheckpoint = 0;
    for (int i = 0; i < timeout; i++) {
      if (!keepGoing.getAsBoolean()) {
        robot.stop();
        return DriveResult.ABORTED;
      }
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
      double speed = Config.MAX_SPEED;
      if (watchAhead) {
        double ahead = robot.frontObstacleCm();
        if (ahead < here.distanceTo(target)) { // something is in the way before the waypoint
          if (ahead <= Config.SONAR_STOP_CM) {
            robot.stop();
            stats.recordObstacleStop();
            Log.warn(String.format(Locale.ROOT, "sonar: obstacle %.0f cm ahead at %s while driving "
                + "to %s; stopping and skipping to the next leg", ahead, here, target));
            return DriveResult.OBSTACLE_AHEAD;
          }
          speed *= clamp((ahead - Config.SONAR_STOP_CM)
              / (Config.SONAR_SLOW_CM - Config.SONAR_STOP_CM), Config.SONAR_MIN_SPEED_FRACTION, 1);
        }
      }
      // Positive error = target is to the left, so speed up the right wheel.
      robot.drive(
          clamp(speed - correction, -Config.MAX_SPEED, Config.MAX_SPEED),
          clamp(speed + correction, -Config.MAX_SPEED, Config.MAX_SPEED));
      robot.step();

      if (++stepsSinceCheckpoint >= Config.STALL_WINDOW_STEPS) {
        Vec2 now = robot.positionCm();
        if (now.distanceTo(checkpoint) < Config.STALL_MIN_PROGRESS_CM) {
          robot.stop();
          stats.recordDriveStall();
          Log.warn("stalled at " + now + " while driving to " + target + "; skipping to next leg");
          return DriveResult.STALLED;
        }
        checkpoint = now;
        stepsSinceCheckpoint = 0;
      }
    }
    robot.stop();
    stats.recordDriveTimeout();
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
