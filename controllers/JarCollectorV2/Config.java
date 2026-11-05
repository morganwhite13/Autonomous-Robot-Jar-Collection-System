/**
 * Every tunable number in one place.
 *
 * <p>Values marked "v1" are copied unchanged from ProjectController3 so v2 keeps the behaviour
 * that was tuned in the simulator. Values marked "new" control v2-only behaviour; each one says
 * how to switch it off to get the v1 behaviour back.
 *
 * <p>Units: wheel speeds in rad/s (Webots motor velocity), distances in cm, angles in degrees,
 * durations in control steps (one step = WorldInfo.basicTimeStep, 32 ms by default).
 */
final class Config {
  private Config() {}

  // ------------------------------------------------------------------ mission
  /** v1: jarsToGo = 5. */
  static final int JAR_COUNT = 5;
  /** new: give up after this many failed searches in a row (v1 retried forever). */
  static final int MAX_CONSECUTIVE_FAILED_SEARCHES = 3;
  /** new: seed for the random wander so runs are repeatable (v1 used unseeded Math.random()). */
  static final long RANDOM_SEED = 2025L;
  /**
   * new, off by default: carry and return along the shorter A*-planned routes
   * ({@link Routes#PLANNED}) instead of v1's hand-tuned ones. In Webots this delivered all five
   * jars in 788 s instead of 879 s (tools/run-in-webots.sh v2-planned). Off by default so that v2
   * keeps every one of v1's waypoints.
   */
  static final boolean USE_PLANNED_ROUTES = false;
  /** v1 printed front-sonar readings 10 times at start-up; v2 prints one line. */
  static final boolean PRINT_SENSOR_SELF_TEST = true;
  static final int SENSOR_SELF_TEST_STEPS = 10;

  // ------------------------------------------------------------------ driving
  /** v1 MAX_SPEED. Wheel angular velocity, about 0.49 m/s of forward speed on a Pioneer 3-DX. */
  static final double MAX_SPEED = 5.0;
  /** v1: route turns stop within +/-3 degrees. */
  static final double HEADING_TOLERANCE_DEG = 3.0;
  /** new: turn at full speed above this error, then slow down proportionally. */
  static final double TURN_SLOWDOWN_DEG = 30.0;
  /**
   * new: slowest wheel speed used while finishing a turn. Set to MAX_SPEED for v1-style
   * bang-bang turns.
   */
  static final double TURN_MIN_SPEED = 0.6;
  /**
   * new: while a jar is held (carrying it to the drop-off and placing it), route turns are capped
   * at this wheel speed. In Webots, turning in place at MAX_SPEED (as v1 did) throws the jar out
   * of the gripper within about 0.6 s; MAX_SPEED / 2 and MAX_SPEED / 3 both keep it. Driving
   * straight at full speed and stopping hard both keep it too. Set to MAX_SPEED for v1 behaviour.
   */
  static final double CARRY_TURN_SPEED = MAX_SPEED / 3;
  /** new: abandon a turn that hasn't converged after this many steps (v1 could loop forever). */
  static final int TURN_TIMEOUT_STEPS = 400;

  /** new: heading-hold gain while driving, wheel rad/s per degree of error. 0 = v1 open loop. */
  static final double HEADING_HOLD_GAIN = 0.1;
  /** new: cap on the heading-hold wheel-speed correction. */
  static final double HEADING_HOLD_MAX_CORRECTION = 1.0;
  /** new: stop steering this close to the waypoint (bearing becomes noisy). */
  static final double HEADING_HOLD_MIN_DISTANCE_CM = 20.0;
  /** new: a drive that moves less than STALL_MIN_PROGRESS_CM in STALL_WINDOW_STEPS is stalled. */
  static final int STALL_WINDOW_STEPS = 40;
  static final double STALL_MIN_PROGRESS_CM = 2.0;
  /** new: hard cap on a single drive segment (v1 had none). */
  static final double DRIVE_TIMEOUT_STEPS_PER_CM = 3.0;
  static final int DRIVE_TIMEOUT_MIN_STEPS = 200;

  /** v1 rotateByDegrees / backUpDistance / moveRandomly all used MAX_SPEED / 3. */
  static final double MANOEUVRE_SPEED = MAX_SPEED / 3;
  /** v1 moveRandomly: turn by a random angle in [-90, 90), then drive forward 60 steps. */
  static final int WANDER_MAX_TURN_DEG = 90;
  static final int WANDER_STEPS = 60;

  // ------------------------------------------------------------------ vision
  /** v1 countColor: a pixel is "green" when R < 120, G > 35 and B < 120. */
  static final int GREEN_MAX_RED = 120;
  static final int GREEN_MIN_GREEN = 35;
  static final int GREEN_MAX_BLUE = 120;
  /**
   * new: green must also beat red and blue by this margin. Without it, v1 counted dark grey
   * (50, 50, 50) and dark brown (100, 80, 60) pixels as jar-green.
   * Set to {@link Vision#V1_COMPATIBLE_MARGIN} to reproduce v1 exactly.
   */
  static final int GREEN_DOMINANCE_MARGIN = 10;
  /** v1 EPSILON: a sector must lead the others by this many pixels to steer towards it. */
  static final int STEER_MARGIN_PIXELS = 10;
  /** v1: drive straight if no sector clearly leads but at least this many green pixels show. */
  static final int STEER_STRAIGHT_MIN_PIXELS = 30;

  // ------------------------------------------------------------------ search (360-degree scan)
  /** v1: rotate at MAX_SPEED / 4 while scanning. */
  static final double SCAN_SPEED = MAX_SPEED / 4;
  /** v1: analyse a frame every 5 steps. */
  static final int SCAN_SAMPLE_EVERY_STEPS = 5;
  /** new: stop once the compass says we turned this far (v1: a fixed 200 steps, about 270 deg). */
  static final double SCAN_SWEEP_DEG = 370.0;
  static final int SCAN_TIMEOUT_STEPS = 600;
  /** v1: a frame counts as a detection with more than 20 green pixels. */
  static final int SCAN_MIN_GREEN_PIXELS = 20;
  /** v1: detections with the jar in the centre third get +50 score. */
  static final int SCAN_CENTER_BONUS = 50;
  /** v1 maxFullRotations. */
  static final int MAX_SCANS_PER_SEARCH = 3;

  // ------------------------------------------------------------------ approach
  /** v1: approach at MAX_SPEED / 8. */
  static final double APPROACH_SPEED = MAX_SPEED / 8;
  /** v1: inner/outer wheel factors when steering towards the jar. */
  static final double APPROACH_INNER_FACTOR = 0.3;
  static final double APPROACH_OUTER_FACTOR = 1.2;
  /**
   * new: hard cap on one approach. v1 aborted after 300 steps ("probably hitting wall"; its
   * separate 400-step limit could never trigger). 300 steps at APPROACH_SPEED is only about 58 cm,
   * while jars start 50-160 cm from the search point, so most first approaches timed out.
   * v2 detects the wall case directly (below) and allows ~175 cm. Set to 300 for v1 behaviour.
   */
  static final int APPROACH_TIMEOUT_STEPS = 900;
  /** new: abort the approach if it moved less than this in the window (pushing against a wall). */
  static final int APPROACH_STALL_WINDOW_STEPS = 60;
  static final double APPROACH_STALL_MIN_PROGRESS_CM = 2.0;
  /** v1: fewer than 15 green pixels counts as "jar not visible". */
  static final int APPROACH_LOST_PIXELS = 15;
  /** v1: abort after more than 30 consecutive steps without the jar. */
  static final int APPROACH_MAX_STEPS_WITHOUT_JAR = 30;
  /** v1: pause 5 steps when the jar position is unclear. */
  static final int REASSESS_PAUSE_STEPS = 5;
  /** v1: back up 50 steps after a failed approach. */
  static final int BACKUP_AFTER_FAILED_APPROACH_STEPS = 50;

  // ------------------------------------------------------------------ gripper (all v1)
  static final double GRIPPER_SPEED = 0.1;
  static final double FINGERS_OPEN = 0.099;
  static final double FINGERS_CLOSED = 0.01;
  static final double LIFT_DOWN = 0.001;
  static final double LIFT_UP = -0.0499;
  static final int STARTUP_SETTLE_MS = 1000;
  static final int GRAB_SETTLE_STEPS = 10;
  static final int GRAB_CLOSE_STEPS = 100;
  static final int GRAB_LIFT_STEPS = 120;
  static final int GRAB_HOLD_STEPS = 20;
  static final int RELEASE_STEPS = 50;
  static final int BACKUP_AFTER_RELEASE_STEPS = 50;

  // ------------------------------------------------------------------ sonar watch (new)
  /**
   * new, off by default: on travel legs (not the box-clearing route, which has to push boxes,
   * and not placement), slow down for anything the two front sonars see before the waypoint, and
   * stop short of it instead of driving into it. v1 printed the sonars once and never used them.
   * Off until it has been tuned in Webots.
   */
  static final boolean SONAR_WATCH = false;
  /** new: start slowing down when an echo is this close (cm from the sonars). */
  static final double SONAR_SLOW_CM = 100;
  /** new: stop and skip to the next waypoint when an echo is this close. */
  static final double SONAR_STOP_CM = 25;
  /** new: slowest speed while slowing down, as a fraction of MAX_SPEED. */
  static final double SONAR_MIN_SPEED_FRACTION = 0.2;

  // ------------------------------------------------------------------ grab check (new)
  /**
   * new, off by default: after lifting, check that the fingers are squeezing something and
   * release and try again if not. v1 printed "Verifying grab..." but checked nothing, so a jar
   * that slipped out was "delivered" anyway. Every grab logs its grip effort either way: in
   * Webots, real grabs measured 1.00 and a jar thrown out of the gripper dropped it to 0. Off by
   * default because no real grab has missed yet, so it has never had to act.
   */
  static final boolean VERIFY_GRABS = false;
  /**
   * new: grip effort (finger motor force as a fraction of its maximum) that counts as holding a
   * jar. Fingers closing on a jar stop against it and push at their limit (about 1); fingers that
   * close on nothing reach their target and stop pushing (about 0).
   */
  static final double GRIP_EFFORT_HOLDING = 0.5;
}
