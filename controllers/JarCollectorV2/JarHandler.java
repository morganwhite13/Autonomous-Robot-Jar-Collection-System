import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Random;
import java.util.function.BooleanSupplier;

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
 *   <li>The approach drives faster while the jar is still far away ({@link
 *       Config#FAST_APPROACH_SPEED}), judged from the lowest image row it shows in.</li>
 *   <li>A full scan also maps where each jar seen is; later searches head straight for the
 *       nearest one instead of scanning again ({@link Config#REMEMBER_JARS}).</li>
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
  private final boolean verifyGrabs;
  /** Where jars were seen in the last full scan and haven't been picked up since. */
  private final List<Vec2> rememberedJars = new ArrayList<>();
  /** Where the last approach was stopped before reaching the jar, and its heading; or null. */
  private Vec2 blockedAt;
  private double blockedHeadingDeg;

  JarHandler(RobotPlatform robot, Navigator nav, MissionStats stats, Random random) {
    this(robot, nav, stats, random, Config.VERIFY_GRABS);
  }

  JarHandler(RobotPlatform robot, Navigator nav, MissionStats stats, Random random,
      boolean verifyGrabs) {
    this.robot = robot;
    this.nav = nav;
    this.stats = stats;
    this.random = random;
    this.verifyGrabs = verifyGrabs;
  }

  /** v1 intelligentJarSearch: up to 3 scan-and-approach attempts. True once a jar is held. */
  boolean searchAndGrab() {
    if (Config.REMEMBER_JARS && grabRememberedJar()) {
      return true;
    }
    for (int attempt = 1; attempt <= Config.MAX_SCANS_PER_SEARCH; attempt++) {
      Log.info("Search attempt " + attempt + "/" + Config.MAX_SCANS_PER_SEARCH);
      stats.recordScan();
      Detection best = scan();
      if (best == null) {
        Log.info("No jar seen in a full turn; wandering to a new spot");
        wander();
        continue;
      }
      Log.info(String.format(Locale.ROOT, "Best detection at %.0f deg: %d px %s (%s)",
          best.headingDeg, best.counts.total(), best.counts, best.counts.dominant()));
      nav.turnTo(best.headingDeg, Config.MANOEUVRE_SPEED);
      stats.recordApproach();
      if (approachAndGrab() || approachAgainAround(rememberedJarTowards(best.headingDeg))) {
        forgetGrabbedJar();
        return true;
      }
      Log.info("Approach failed; backing up");
      nav.driveSteps(-Config.MANOEUVRE_SPEED, Config.BACKUP_AFTER_FAILED_APPROACH_STEPS);
    }
    return false;
  }

  /**
   * Turns to the nearest jar remembered from an earlier scan and, if the camera sees it there,
   * approaches and grabs it. A jar that isn't where it was seen is forgotten.
   */
  private boolean grabRememberedJar() {
    while (!rememberedJars.isEmpty()) {
      Vec2 here = robot.positionCm();
      Vec2 target = rememberedJars.get(0);
      for (Vec2 jar : rememberedJars) {
        if (here.distanceTo(jar) < here.distanceTo(target)) {
          target = jar;
        }
      }
      Log.info("Heading for the jar remembered at " + target);
      nav.turnTo(here.bearingTo(target), Config.MANOEUVRE_SPEED);
      robot.step(); // a fresh frame from the new heading
      Vision.SectorCounts counts = Vision.countGreen(robot.captureImage());
      if (counts.total() > Config.SCAN_MIN_GREEN_PIXELS) {
        stats.recordApproach();
        if (approachAndGrab() || approachAgainAround(target)) {
          forgetGrabbedJar();
          return true;
        }
        Log.info("Approach failed; backing up");
        nav.driveSteps(-Config.MANOEUVRE_SPEED, Config.BACKUP_AFTER_FAILED_APPROACH_STEPS);
      } else {
        Log.info("No jar where one was remembered");
      }
      rememberedJars.remove(target);
    }
    return false;
  }

  /** After a grab: the held jar is just ahead of the robot; forget remembered jars there. */
  private void forgetGrabbedJar() {
    Vec2 here = robot.positionCm();
    double h = Math.toRadians(robot.headingDeg());
    double reach = Config.CAMERA_AHEAD_CM + 2 * Config.JAR_RADIUS_CM + 7; // bumper to jar centre
    Vec2 held = Vec2.of(here.x + reach * Math.cos(h), here.y + reach * Math.sin(h));
    rememberedJars.removeIf(jar -> jar.distanceTo(held) <= Config.JAR_MAP_FORGET_CM);
  }

  /**
   * After an approach that something stopped before the jar at {@code jar} (null: the jar isn't
   * on the map, or the approach ended some other way): go around what stopped it, to {@link
   * Config#APPROACH_AGAIN_FROM_CM} from the jar, and approach once more. True once it's held.
   */
  private boolean approachAgainAround(Vec2 jar) {
    if (blockedAt == null || jar == null) {
      return false;
    }
    Log.info("Something stopped the approach at " + blockedAt + "; going around it to the jar at "
        + jar);
    Navigator.DriveResult around = nav.goAround(jar, blockedAt, blockedHeadingDeg,
        new Navigator.Travel().keepGoingWhile(
            () -> robot.positionCm().distanceTo(jar) > Config.APPROACH_AGAIN_FROM_CM));
    if (around != Navigator.DriveResult.ABORTED && around != Navigator.DriveResult.ARRIVED) {
      return false;
    }
    nav.turnTo(robot.positionCm().bearingTo(jar), Config.MANOEUVRE_SPEED);
    stats.recordApproach();
    return approachAndGrab();
  }

  /** The remembered jar closest in bearing to {@code headingDeg}, within 20 degrees, or null. */
  private Vec2 rememberedJarTowards(double headingDeg) {
    Vec2 here = robot.positionCm();
    Vec2 best = null;
    double bestOff = 20;
    for (Vec2 jar : rememberedJars) {
      double off = Math.abs(Angles.delta(headingDeg, here.bearingTo(jar)));
      if (off <= bestOff) {
        bestOff = off;
        best = jar;
      }
    }
    return best;
  }

  /** Jars currently remembered, nearest first is not guaranteed. */
  List<Vec2> rememberedJars() {
    return java.util.Collections.unmodifiableList(rememberedJars);
  }

  /**
   * Rotates one full turn (measured by the compass) and returns the best-scoring detection. With
   * {@link Config#REMEMBER_JARS}, also replaces the remembered jars with the ones seen.
   */
  Detection scan() {
    Log.info("Scanning 360 degrees for green");
    List<Vec2> sightings = new ArrayList<>();
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
        RgbImage image = robot.captureImage();
        if (Config.REMEMBER_JARS) {
          sight(image, heading, sightings);
        }
        Vision.SectorCounts counts = Vision.countGreen(image);
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
    if (Config.REMEMBER_JARS) {
      rememberedJars.clear();
      rememberedJars.addAll(cluster(sightings));
      if (!rememberedJars.isEmpty()) {
        Log.info("Jars seen in the scan: " + rememberedJars);
      }
    }
    return best;
  }

  /** Adds where each jar in this frame appears to be, from the robot's pose. */
  private void sight(RgbImage image, double headingDeg, List<Vec2> sightings) {
    Vec2 here = robot.positionCm();
    for (Vision.Blob blob : Vision.blobs(image)) {
      double distance = Vision.distanceCm(blob.lowestRow);
      // Only whole jars: one touching the frame's edge or bottom would be misjudged.
      if (blob.fromColumn == 0 || blob.toColumn == image.width - 1
          || blob.lowestRow >= image.height - 1 || Double.isInfinite(distance)) {
        continue;
      }
      double bearing = Math.toRadians(headingDeg + Vision.bearingOffsetDeg(blob.centreColumn()));
      sightings.add(Vec2.of(here.x + distance * Math.cos(bearing),
          here.y + distance * Math.sin(bearing)));
    }
  }

  /** Groups sightings within JAR_MAP_MERGE_CM; keeps groups seen often enough, averaged. */
  static List<Vec2> cluster(List<Vec2> sightings) {
    List<List<Vec2>> groups = new ArrayList<>();
    for (Vec2 s : sightings) {
      List<Vec2> home = null;
      for (List<Vec2> g : groups) {
        if (centre(g).distanceTo(s) <= Config.JAR_MAP_MERGE_CM) {
          home = g;
          break;
        }
      }
      if (home == null) {
        home = new ArrayList<>();
        groups.add(home);
      }
      home.add(s);
    }
    List<Vec2> jars = new ArrayList<>();
    for (List<Vec2> g : groups) {
      if (g.size() >= Config.JAR_MAP_MIN_SIGHTINGS) {
        jars.add(centre(g));
      }
    }
    return jars;
  }

  private static Vec2 centre(List<Vec2> points) {
    double x = 0;
    double y = 0;
    for (Vec2 p : points) {
      x += p.x;
      y += p.y;
    }
    return Vec2.of(x / points.size(), y / points.size());
  }

  /**
   * v1 carefullyApproachGreen: creep forward, steering towards the greenest third of the image,
   * until the bumper is pressed (then grab). Gives up if the jar stays out of sight, the robot
   * stops making progress, or {@link Config#APPROACH_TIMEOUT_STEPS} pass.
   */
  boolean approachAndGrab() {
    Log.info("Approaching jar");
    blockedAt = null;
    int stepsWithoutJar = 0;
    Vec2 checkpoint = robot.positionCm();
    for (int step = 1; step <= Config.APPROACH_TIMEOUT_STEPS; step++) {
      robot.step();
      if (robot.jarTouched()) {
        return grab();
      }

      RgbImage image = robot.captureImage();
      Vision.SectorCounts counts = Vision.countGreen(image);
      stepsWithoutJar = counts.total() < Config.APPROACH_LOST_PIXELS ? stepsWithoutJar + 1 : 0;
      if (stepsWithoutJar > Config.APPROACH_MAX_STEPS_WITHOUT_JAR) {
        stopApproach("Lost sight of the jar; aborting approach (" + v1Count(image) + ")");
        return false;
      }
      if (step % 20 == 0) {
        Log.info("  approach step " + step + ": " + counts.total() + " px " + counts
            + " (" + v1Count(image) + ")");
      }

      Vision.Steering steering = Vision.steer(counts);
      if (steering == Vision.Steering.UNCLEAR && Config.STEER_WHEN_UNCLEAR
          && counts.total() >= Config.APPROACH_LOST_PIXELS) {
        steering = Vision.lean(counts);
      }
      if (steering != Vision.Steering.UNCLEAR) {
        steer(steering, Vision.distanceCm(Vision.lowestGreenRow(image)));
      } else if (pauseToReassess()) {
        return grab();
      }

      if (step % Config.APPROACH_STALL_WINDOW_STEPS == 0) {
        Vec2 now = robot.positionCm();
        if (now.distanceTo(checkpoint) < Config.APPROACH_STALL_MIN_PROGRESS_CM) {
          blockedAt = now;
          blockedHeadingDeg = robot.headingDeg();
          stopApproach("Not making progress (pushing against something); aborting approach");
          return false;
        }
        checkpoint = now;
      }
    }
    stopApproach("Approach timed out");
    return false;
  }

  /**
   * v1's wheel speeds for each steering decision (UNCLEAR is handled by the caller). While the
   * jar is still far away the same extra speed is added to both wheels: the robot closes in
   * faster but turns no more sharply than v1's creep, so the jar stays in the camera's view.
   */
  private void steer(Vision.Steering steering, double jarDistanceCm) {
    double s = Config.APPROACH_SPEED;
    double boost = jarDistanceCm > Config.FAST_APPROACH_UNTIL_CM
        ? Config.FAST_APPROACH_SPEED - Config.APPROACH_SPEED : 0;
    double inner = s * Config.APPROACH_INNER_FACTOR + boost;
    double outer = s * Config.APPROACH_OUTER_FACTOR + boost;
    switch (steering) {
      case LEFT:
        robot.drive(inner, outer);
        break;
      case RIGHT:
        robot.drive(outer, inner);
        break;
      case STRAIGHT:
        robot.drive(s + boost, s + boost);
        break;
      default:
        throw new IllegalArgumentException("no wheel speeds for " + steering);
    }
  }

  /** v1's "reassess" pause: stand still for a moment. Returns true if the bumper is pressed. */
  private boolean pauseToReassess() {
    robot.stop();
    for (int i = 0; i < Config.REASSESS_PAUSE_STEPS; i++) {
      robot.step();
      if (robot.jarTouched()) {
        return true;
      }
    }
    return false;
  }

  private void stopApproach(String reason) {
    robot.stop();
    Log.info(reason);
  }

  /**
   * Diagnostic for the first Webots runs: what v1's looser green test would count in the same
   * frame. v2's pixel thresholds were tuned on v1's counts; if approaches end with "Lost sight"
   * a few cm short of the jar while this number stays high, set GREEN_DOMINANCE_MARGIN to
   * Vision.V1_COMPATIBLE_MARGIN.
   */
  private static String v1Count(RgbImage image) {
    return "v1 rule: " + Vision.countGreen(image, Vision.V1_COMPATIBLE_MARGIN).total() + " px";
  }

  /**
   * v1 executeGrab: same positions and timings. With the grab check on, a grab whose fingers
   * aren't squeezing anything afterwards is undone and counts as a failed attempt.
   */
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
    robot.setLift(Config.CARRY_LIFT);
    robot.waitSteps(Config.GRAB_LIFT_STEPS);
    robot.waitSteps(Config.GRAB_HOLD_STEPS);
    // v1 printed "Verifying grab..." but did not check anything. v2 logs what the camera sees
    // and how hard the fingers squeeze, so the check below can be calibrated from real runs.
    double effort = robot.gripEffort();
    String effortText = Double.isNaN(effort) ? "not measured"
        : String.format(Locale.ROOT, "%.2f", effort);
    Log.info("Grab sequence complete (camera now sees "
        + Vision.countGreen(robot.captureImage()).total() + " green px, grip effort " + effortText
        + ")");
    if (verifyGrabs && effort < Config.GRIP_EFFORT_HOLDING) { // NaN: can't tell, assume held
      Log.warn(String.format(Locale.ROOT, "the fingers aren't holding anything (grip effort "
          + "%.2f); releasing and trying again", effort));
      stats.recordMissedGrab();
      robot.setFingers(Config.FINGERS_OPEN);
      robot.setLift(Config.LIFT_DOWN);
      robot.waitSteps(Config.RELEASE_STEPS);
      return false;
    }
    return true;
  }

  /**
   * Watches the finger force while a jar is carried ({@link Config#WATCH_GRIP_WHILE_CARRYING}):
   * false once it has stayed below {@link Config#GRIP_EFFORT_HOLDING} for {@link
   * Config#DROP_CONFIRM_STEPS} checks in a row. A platform that can't measure it (NaN) counts as
   * holding.
   */
  final class GripWatch implements BooleanSupplier {
    private int lowFor;
    private double lowest = Double.NaN;

    @Override
    public boolean getAsBoolean() {
      if (!Config.WATCH_GRIP_WHILE_CARRYING) {
        return true;
      }
      double effort = robot.gripEffort();
      if (!(effort >= lowest)) { // also true while lowest is NaN
        lowest = Double.isNaN(effort) ? lowest : effort;
      }
      lowFor = effort < Config.GRIP_EFFORT_HOLDING ? lowFor + 1 : 0;
      return lowFor < Config.DROP_CONFIRM_STEPS;
    }

    /** The lowest grip effort seen so far, or NaN if none was measured. */
    double lowest() {
      return lowest;
    }
  }

  GripWatch gripWatch() {
    return new GripWatch();
  }

  /** v1 placeJar: drive into the slot for this jar, open and lower the gripper, back out. */
  void place(int jarIndex) {
    place(jarIndex, () -> true);
  }

  /**
   * As {@link #place(int)}, but returns false, without backing out, if {@code holding} reports
   * the jar gone on the way to the slot.
   */
  boolean place(int jarIndex, BooleanSupplier holding) {
    Log.info("Placing jar in slot " + (jarIndex + 1) + " (y=" + Routes.slotY(jarIndex) + ")");
    // Raised to v1's height for the short run in, so the jar rides onto the pad as in v1.
    robot.setLift(Config.LIFT_UP);
    Navigator.RouteResult result = nav.drive(Routes.placementPath(jarIndex),
        new Navigator.Travel().turnSpeed(Config.CARRY_TURN_SPEED).keepGoingWhile(holding));
    robot.setFingers(Config.FINGERS_OPEN);
    robot.setLift(Config.LIFT_DOWN);
    robot.waitSteps(Config.RELEASE_STEPS);
    if (result == Navigator.RouteResult.ABORTED) {
      return false;
    }
    nav.driveSteps(-Config.MANOEUVRE_SPEED, Config.BACKUP_AFTER_RELEASE_STEPS);
    return true;
  }

  /** v1 moveRandomly: turn by a random angle in [-90, 90), then drive forward briefly. */
  void wander() {
    int turn = random.nextInt(2 * Config.WANDER_MAX_TURN_DEG) - Config.WANDER_MAX_TURN_DEG;
    nav.turnTo(Angles.normalize(robot.headingDeg() + turn), Config.MANOEUVRE_SPEED);
    nav.driveSteps(Config.MANOEUVRE_SPEED, Config.WANDER_STEPS);
  }
}
