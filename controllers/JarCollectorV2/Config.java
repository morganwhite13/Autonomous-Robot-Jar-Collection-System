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
   * new: carry and return along the shorter A*-planned routes ({@link Routes#PLANNED}) instead
   * of v1's hand-tuned ones. On six shuffled Webots worlds the median mission took 740 s instead
   * of 837 s. Set to false for v1's waypoints (tools/run-in-webots.sh v2-hand-tuned).
   */
  static final boolean USE_PLANNED_ROUTES = true;
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
  /**
   * new: when the jar is in sight but neither rule above applies, steer towards the outer third
   * with more green (straight if they're level) instead of v1's stop-and-reassess. A small, far
   * jar split across two thirds (say 5 px in the centre and 15 on the right) never looks any
   * different while the robot stands still, so v1 stood there until the approach gave up. In
   * Webots this stopped every approach to the last jar on one shuffled world. Set to false for
   * v1 behaviour.
   */
  static final boolean STEER_WHEN_UNCLEAR = true;

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
  /**
   * new: approach at this speed while the jar is estimated to be more than FAST_APPROACH_UNTIL_CM
   * away, then creep at APPROACH_SPEED as v1 did all the way. Set FAST_APPROACH_SPEED to
   * APPROACH_SPEED for v1 behaviour.
   */
  static final double FAST_APPROACH_SPEED = MAX_SPEED / 3;
  static final double FAST_APPROACH_UNTIL_CM = 75;
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
  /** v1: lift height after a grab. v2 uses it only for the short run onto the drop-off pad. */
  static final double LIFT_UP = -0.0499;
  /**
   * new: how high v2 lifts a jar to carry it. In Webots a jar lifted to LIFT_UP (5 cm) often
   * slipped out of the fingers on hard stops and turns (1 of 3 kept through four 4 m runs and
   * 180 degree turns); lifted 0.8 cm it was kept in 6 of 6, including grabs 4 and 7 cm off
   * centre, and any slip sets it down upright between the fingers. Set to LIFT_UP for v1
   * behaviour.
   */
  static final double CARRY_LIFT = -0.008;
  static final int STARTUP_SETTLE_MS = 1000;
  static final int GRAB_SETTLE_STEPS = 10;
  static final int GRAB_CLOSE_STEPS = 100;
  static final int GRAB_LIFT_STEPS = 120;
  static final int GRAB_HOLD_STEPS = 20;
  static final int RELEASE_STEPS = 50;
  static final int BACKUP_AFTER_RELEASE_STEPS = 50;

  // ------------------------------------------------------------------ sonar watch (new)
  /**
   * new: on travel legs (not the box-clearing route, which has to push boxes, and not
   * placement), slow down for anything the two front sonars see before the waypoint, and stop
   * short of it instead of driving into it. v1 printed the sonars once and never used them. In
   * Webots it never stopped for nothing on six shuffled worlds (the runs matched the ones without
   * it), and with a wooden box on the carry route it stopped 24 cm short and drove around it.
   */
  static final boolean SONAR_WATCH = true;
  /** new: start slowing down when an echo is this close (cm from the sonars). */
  static final double SONAR_SLOW_CM = 100;
  /** new: stop and skip to the next waypoint when an echo is this close. */
  static final double SONAR_STOP_CM = 25;
  /** The front sonars (so3, so4) sit this far ahead of the robot's centre (Pioneer3dx proto). */
  static final double FRONT_SONAR_AHEAD_CM = 16.4;
  /** new: slowest speed while slowing down, as a fraction of MAX_SPEED. */
  static final double SONAR_MIN_SPEED_FRACTION = 0.2;

  // ------------------------------------------------------------------ grab check (new)
  /**
   * new: after lifting, check that the fingers are squeezing something and release and try
   * again if not. v1 printed "Verifying grab..." but checked nothing, so a jar that slipped out
   * was "delivered" anyway. In Webots, real grabs measured 1.00 and a jar thrown out of the
   * gripper dropped it to 0; on six shuffled worlds the check never rejected a real grab.
   */
  static final boolean VERIFY_GRABS = true;
  /**
   * new: grip effort (finger motor force as a fraction of its maximum) that counts as holding a
   * jar. Fingers closing on a jar stop against it and push at their limit (about 1); fingers that
   * close on nothing reach their target and stop pushing (about 0).
   */
  static final double GRIP_EFFORT_HOLDING = 0.5;

  // ------------------------------------------------------------------ carrying (new)
  /**
   * new: while carrying a jar, watch the finger force. If it stays below GRIP_EFFORT_HOLDING for
   * DROP_CONFIRM_STEPS steps in a row, the jar has fallen out: v2 stops, goes back to the search
   * point and carries on with the next jar. v1 drove on to the drop-off empty-handed and counted
   * the delivery. Set to false for v1 behaviour.
   */
  static final boolean WATCH_GRIP_WHILE_CARRYING = true;
  /**
   * new: in Webots the force dips below the threshold for up to 9 steps while the jar is still
   * held, and stays down for good once it has fallen out; 30 steps (about 1 s) tells them apart.
   */
  static final int DROP_CONFIRM_STEPS = 30;
  // ------------------------------------------------------------------ detours (new)
  /**
   * new: when a leg of a travel route stalls (not the box-clearing route, which is meant to push
   * boxes), v2 treats a square just ahead of it as an obstacle, backs off, plans a way around on
   * {@link ArenaMap} with A*, and follows that. If there's no way, it skips to the next waypoint
   * as before. v1 pushed forever. Set to false to only skip.
   */
  static final boolean DETOUR_AROUND_STALLS = true;
  /** new: the assumed obstacle: a square this wide, centred this far ahead of the robot. */
  static final double DETOUR_BLOCKER_SIZE_CM = 40;
  static final double DETOUR_BLOCKER_AHEAD_CM = 35;
  /** new: back off this long (at MANOEUVRE_SPEED) before following the detour. */
  static final int DETOUR_BACKUP_STEPS = 25;
  /** new: how far from the robot (or a waypoint near a wall) to look for open floor to plan from. */
  static final double DETOUR_FIND_FREE_CM = 80;
  // ------------------------------------------------------------------ camera geometry (new)
  /**
   * new: where the camera is, for estimating how far away a jar is from the lowest row it shows
   * in (ground-plane geometry: the camera looks straight ahead, so a jar's base appears higher in
   * the image the further away it is). Measured in Webots with a jar at known distances: the
   * estimate is within 5 cm out to 1 m and within 15 cm out to 1.5 m.
   */
  static final double CAMERA_HEIGHT_CM = 16.5;
  /** Camera ahead of the robot's centre. */
  static final double CAMERA_AHEAD_CM = 14;
  /** Pixels per unit of tan(angle): 32 / tan(22.5 deg) for the 64-pixel, 45-degree camera. */
  static final double CAMERA_FOCAL_PX = 77.25;
  /** Honey jar radius (the proto's collision cylinder). */
  static final double JAR_RADIUS_CM = 4.5;
  // ------------------------------------------------------------------ jar map (new)
  /**
   * new: during a full scan, record where each jar seen appears to be. Later searches from the
   * search point turn straight to the nearest remembered jar and check it is there, instead of
   * scanning a full turn again. If it isn't, they fall back to the scan. Set to false for v1
   * behaviour (scan every time).
   */
  static final boolean REMEMBER_JARS = true;
  /** new: sightings this close together are the same jar. */
  static final double JAR_MAP_MERGE_CM = 25;
  /** new: a jar must be seen in at least this many scan frames to be remembered. */
  static final int JAR_MAP_MIN_SIGHTINGS = 2;
  /** new: after a grab, forget remembered jars this close to where the grabbed jar was. */
  static final double JAR_MAP_FORGET_CM = 30;
  /**
   * new: when something stops an approach before the jar (it isn't making progress) and the jar
   * is on the map, drive around what stopped it (as for a stalled leg, {@link
   * #DETOUR_AROUND_STALLS}) to this far from the jar and approach again from there. In Webots,
   * on one shuffled world, approaches to the last jar from the search point were stopped dead
   * again and again while a slightly different line got through. v1 had no jar map.
   */
  static final double APPROACH_AGAIN_FROM_CM = 80;
}
