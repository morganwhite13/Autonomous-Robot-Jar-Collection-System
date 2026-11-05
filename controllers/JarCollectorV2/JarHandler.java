import java.util.Locale;
import java.util.Random;

/**
 * Finding, grabbing and placing jars. The algorithm is v1's (scan, turn to best detection,
 * approach with visual servoing until the bumper is pressed, grip, lift) with these changes:
 * <ul>
 *   <li>The scan stops when the compass says it has turned 370 degrees. v1 rotated for a fixed
 *       200 steps, which at 32 ms per step and MAX_SPEED / 4 is roughly 270 degrees.</li>
 *   <li>The bumper is also checked during the "reassess" pause (v1 ignored it there).</li>
 *   <li>The approach aborts when the robot stops making progress (what v1's "probably hitting
 *       wall" timeout was guessing at) instead of after a fixed 300 steps, which was only about
 *       58 cm of travel and too short for most jars from the search point.</li>
 *   <li>The random wander uses a seeded generator so runs are repeatable.</li>
 * </ul>
 */
final class JarHandler {

  /** Best green detection seen during a scan. */
  static final class Detection {
    final double headingDeg;
    final Vision.SectorCounts counts;
    final int score;

    Detection(double headingDeg, Vision.SectorCounts counts, int score) {
      this.headingDeg = headingDeg;
      this.counts = counts;
      this.score = score;
    }
  }

  private final RobotPlatform robot;
  private final Navigator nav;
  private final MissionStats stats;
  private final Random random;

  JarHandler(RobotPlatform robot, Navigator nav, MissionStats stats, Random random) {
    this.robot = robot;
    this.nav = nav;
    this.stats = stats;
    this.random = random;
  }

  /** v1 intelligentJarSearch: up to 3 scan-and-approach attempts. True once a jar is held. */
  boolean searchAndGrab() {
    for (int attempt = 1; attempt <= Config.MAX_SCANS_PER_SEARCH; attempt++) {
      Log.info("Search attempt " + attempt + "/" + Config.MAX_SCANS_PER_SEARCH);
      stats.recordScan();
      Detection best = scan();
      if (best == null) {
        Log.info("No jar seen in a full turn; wandering to a new spot");
        wander();
        continue;
      }
      Log.info(String.format(Locale.ROOT, "Best detection at %.0f deg: %d px %s (%s)", best.headingDeg,
          best.counts.total(), best.counts, best.counts.dominant()));
      nav.turnTo(best.headingDeg, Config.MANOEUVRE_SPEED);
      stats.recordApproach();
      if (approachAndGrab()) {
        return true;
      }
      Log.info("Approach failed; backing up");
      nav.driveSteps(-Config.MANOEUVRE_SPEED, Config.BACKUP_AFTER_FAILED_APPROACH_STEPS);
    }
    return false;
  }

  /** Rotates one full turn (measured by the compass) and returns the best-scoring detection. */
  Detection scan() {
    Log.info("Scanning 360 degrees for green");
    Detection best = null;
    double swept = 0;
    double previous = robot.headingDeg();
    robot.drive(-Config.SCAN_SPEED, Config.SCAN_SPEED);
    for (int step = 0; step < Config.SCAN_TIMEOUT_STEPS; step++) {
      robot.step();
      double heading = robot.headingDeg();
      swept += Angles.delta(previous, heading);
      previous = heading;

      if (step % Config.SCAN_SAMPLE_EVERY_STEPS == 0) {
        Vision.SectorCounts counts = Vision.countGreen(robot.captureImage());
        if (counts.total() > Config.SCAN_MIN_GREEN_PIXELS) {
          int score = Vision.scanScore(counts);
          if (best == null || score > best.score) {
            best = new Detection(heading, counts, score);
          }
        }
      }
      if (Math.abs(swept) >= Config.SCAN_SWEEP_DEG) {
        break;
      }
    }
    robot.stop();
    if (Math.abs(swept) < Config.SCAN_SWEEP_DEG) {
      Log.warn(String.format(Locale.ROOT, "scan timed out after %.0f degrees", Math.abs(swept)));
    }
    return best;
  }

  /** v1 carefullyApproachGreen: creep towards the jar, steering by which third of the image is greenest. */
  boolean approachAndGrab() {
    Log.info("Approaching jar");
    int stepsWithoutJar = 0;
    double s = Config.APPROACH_SPEED;
    Vec2 checkpoint = robot.positionCm();
    for (int step = 1; step <= Config.APPROACH_TIMEOUT_STEPS; step++) {
      robot.step();
      if (robot.jarTouched()) {
        robot.stop();
        return grab();
      }

      Vision.SectorCounts counts = Vision.countGreen(robot.captureImage());
      if (counts.total() < Config.APPROACH_LOST_PIXELS) {
        if (++stepsWithoutJar > Config.APPROACH_MAX_STEPS_WITHOUT_JAR) {
          robot.stop();
          Log.info("Lost sight of the jar; aborting approach");
          return false;
        }
      } else {
        stepsWithoutJar = 0;
      }

      if (step % 20 == 0) {
        Log.info("  approach step " + step + ": " + counts.total() + " px " + counts);
      }

      switch (Vision.steer(counts)) {
        case LEFT:
          robot.drive(s * Config.APPROACH_INNER_FACTOR, s * Config.APPROACH_OUTER_FACTOR);
          break;
        case RIGHT:
          robot.drive(s * Config.APPROACH_OUTER_FACTOR, s * Config.APPROACH_INNER_FACTOR);
          break;
        case STRAIGHT:
          robot.drive(s, s);
          break;
        default:
          robot.stop();
          for (int i = 0; i < Config.REASSESS_PAUSE_STEPS; i++) {
            robot.step();
            if (robot.jarTouched()) {
              return grab();
            }
          }
      }

      if (step % Config.APPROACH_STALL_WINDOW_STEPS == 0) {
        Vec2 now = robot.positionCm();
        if (now.distanceTo(checkpoint) < Config.APPROACH_STALL_MIN_PROGRESS_CM) {
          robot.stop();
          Log.info("Not making progress (pushing against something); aborting approach");
          return false;
        }
        checkpoint = now;
      }
    }
    robot.stop();
    Log.info("Approach timed out");
    return false;
  }

  /** v1 executeGrab: same positions and timings. */
  boolean grab() {
    if (!robot.jarTouched()) {
      Log.info("Lost contact before grabbing");
      return false;
    }
    Log.info("Contact: closing gripper");
    robot.stop();
    robot.waitSteps(Config.GRAB_SETTLE_STEPS);
    robot.setFingers(Config.FINGERS_CLOSED);
    robot.waitSteps(Config.GRAB_CLOSE_STEPS);
    Log.info("Lifting jar");
    robot.setLift(Config.LIFT_UP);
    robot.waitSteps(Config.GRAB_LIFT_STEPS);
    robot.waitSteps(Config.GRAB_HOLD_STEPS);
    // v1 printed "Verifying grab..." but did not check anything. v2 does not claim to verify:
    // it logs what the camera sees so a real check can be calibrated from actual runs.
    Log.info("Grab sequence complete (camera now sees "
        + Vision.countGreen(robot.captureImage()).total() + " green px)");
    return true;
  }

  /** v1 placeJar: drive into the slot for this jar, open and lower the gripper, back out. */
  void place(int jarIndex) {
    Log.info("Placing jar in slot " + (jarIndex + 1) + " (y=" + Routes.slotY(jarIndex) + ")");
    nav.followRoute(Routes.placementPath(jarIndex));
    robot.setFingers(Config.FINGERS_OPEN);
    robot.setLift(Config.LIFT_DOWN);
    robot.waitSteps(Config.RELEASE_STEPS);
    nav.driveSteps(-Config.MANOEUVRE_SPEED, Config.BACKUP_AFTER_RELEASE_STEPS);
  }

  /** v1 moveRandomly: turn by a random angle in [-90, 90), then drive forward briefly. */
  void wander() {
    int turn = random.nextInt(2 * Config.WANDER_MAX_TURN_DEG) - Config.WANDER_MAX_TURN_DEG;
    nav.turnTo(Angles.normalize(robot.headingDeg() + turn), Config.MANOEUVRE_SPEED);
    nav.driveSteps(Config.MANOEUVRE_SPEED, Config.WANDER_STEPS);
  }
}
