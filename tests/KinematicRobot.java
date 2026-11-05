import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Predicate;

/**
 * A test double for {@link RobotPlatform}: an ideal differential-drive robot with the
 * Pioneer 3-DX geometry and a synthetic camera that draws jars as green rectangles.
 *
 * <p>It models kinematics only (no inertia, wheel slip or box pushing), so it tests control
 * logic, not physics. Geometry comes from the Pioneer3dx proto: wheel radius 9.75 cm, wheels at
 * +/-16.5 cm. Camera: Webots default 45 degree field of view, 64 x 64 pixels.
 */
final class KinematicRobot implements RobotPlatform {
  static final double WHEEL_RADIUS_CM = 9.75;
  static final double AXLE_TRACK_CM = 33.0;
  static final double FOV_DEG = 45.0;
  static final int IMAGE_SIZE = 64;
  /** Robot centre to jar centre when the front bumper touches (bumper is 20.4 cm ahead). */
  static final double TOUCH_DISTANCE_CM = 25.0;
  static final double JAR_RADIUS_CM = 5.0;
  static final double JAR_HEIGHT_CM = 15.0;
  static final int FLOOR = RgbImage.pack(190, 150, 110);
  static final int JAR = RgbImage.pack(30, 160, 30);

  private final int timeStepMs;
  private double x;
  private double y;
  private double heading;
  private double leftSpeed;
  private double rightSpeed;
  private double commandedLeft;
  private double commandedRight;
  private final ArrayDeque<double[]> pendingCommands = new ArrayDeque<>();
  /** Steps before a wheel-speed command takes effect (0 = immediately). */
  private int actuatorLagSteps;
  /** Fraction of the ideal turn rate actually achieved (1 = no slip). */
  private double turnEfficiency = 1.0;
  private long elapsedMs;
  private long steps;
  private long maxSteps = 500_000;
  /** Signed total rotation since construction (positive = counter-clockwise). */
  double cumulativeTurnDeg;
  /** If set, the position after every step is appended (used to draw paths). */
  List<Vec2> trace;
  /** If set, the heading after every step is appended, unwrapped (it can pass 360 or go below 0). */
  List<Double> headingTrace;
  /** Simulated time of each jar release, in the same order as {@link #deliveredJars}. */
  final List<Long> deliveryTimesMs = new ArrayList<>();
  private final double startHeadingDeg;
  /** Step count when the wheels were first commanded to move, or -1. */
  long firstMoveStep = -1;

  final List<Vec2> jars = new ArrayList<>();
  final List<Vec2> deliveredJars = new ArrayList<>();
  Vec2 heldJar;
  /** How many of the next grabs miss: the jar slips out of the fingers and stays on the floor. */
  int grabsThatMiss;
  /** True from a missed grab until the fingers open again. */
  private boolean missedThisGrab;
  double fingers = Config.FINGERS_OPEN;
  double lift = Config.LIFT_DOWN;
  /**
   * Gripper positions within this of a target count as reaching it. v1 passes float literals
   * (0.099f, -0.0499f), which reach Webots as doubles a few 1e-9 away from v2's values.
   */
  static final double GRIPPER_TOLERANCE = 1e-6;
  /** Positions the robot centre may not enter (e.g. a wall for stall tests). */
  Predicate<Vec2> blocked = p -> false;
  /** Solid obstacle outlines: they block the robot (see {@link #withStaticObstacles}) and echo sonar. */
  private final List<Vec2[]> solids = new ArrayList<>();
  /** Forward speed (cm/s) the first time an obstacle stopped the robot; NaN if it never touched one. */
  double firstContactSpeedCmS = Double.NaN;

  KinematicRobot(Vec2 position, double headingDeg) {
    this(position, headingDeg, 32);
  }

  KinematicRobot(Vec2 position, double headingDeg, int timeStepMs) {
    this.x = position.x;
    this.y = position.y;
    this.heading = Angles.normalize(headingDeg);
    this.startHeadingDeg = headingDeg;
    this.timeStepMs = timeStepMs;
  }

  /**
   * Collision radius for walls and wooden boxes: half the Pioneer 3-DX's 38 cm width. The body is
   * longer than it is wide, so this under-counts head-on contact; the routes keep 30 cm clear.
   */
  static final double ROBOT_RADIUS_CM = 19;

  /**
   * The mission's starting state: the world's start pose (facing +y: the robot's rotation in the
   * world file is the identity and the Pioneer's forward axis is -Z, which is +y here), the five
   * jars, and walls and wooden boxes made solid.
   */
  static KinematicRobot atMissionStart(WorldFile world) {
    KinematicRobot robot = new KinematicRobot(world.robotStart, 90)
        .withStaticObstacles(world, ROBOT_RADIUS_CM);
    for (WorldFile.Obstacle jar : world.ofKind(WorldFile.Kind.HONEY_JAR)) {
      robot.jars.add(jar.center);
    }
    return robot;
  }

  /** Ends the "simulation" after this many steps so a buggy loop fails the test instead of hanging. */
  KinematicRobot withStepLimit(long limit) {
    this.maxSteps = limit;
    return this;
  }

  /**
   * Makes the model less ideal: wheel commands take effect {@code lagSteps} steps late, and turns
   * only achieve {@code turnEfficiency} of the ideal rate (wheel slip, castor drag).
   */
  KinematicRobot withImperfections(int lagSteps, double turnEfficiency) {
    this.actuatorLagSteps = lagSteps;
    this.turnEfficiency = turnEfficiency;
    return this;
  }

  long stepsTaken() {
    return steps;
  }

  @Override
  public void step() {
    if (steps >= maxSteps) {
      throw new SimulationEnded();
    }
    pendingCommands.addLast(new double[] {commandedLeft, commandedRight});
    if (pendingCommands.size() > actuatorLagSteps) {
      double[] applied = pendingCommands.removeFirst();
      leftSpeed = applied[0];
      rightSpeed = applied[1];
    }
    double dt = timeStepMs / 1000.0;
    double v = WHEEL_RADIUS_CM * (leftSpeed + rightSpeed) / 2;
    double omega = turnEfficiency * WHEEL_RADIUS_CM * (rightSpeed - leftSpeed) / AXLE_TRACK_CM;
    double mid = Math.toRadians(heading) + omega * dt / 2;
    Vec2 next = Vec2.of(x + v * dt * Math.cos(mid), y + v * dt * Math.sin(mid));
    if (!blocked.test(next)) {
      x = next.x;
      y = next.y;
    } else if (Double.isNaN(firstContactSpeedCmS)) {
      firstContactSpeedCmS = Math.abs(v);
    }
    heading = Angles.normalize(heading + Math.toDegrees(omega * dt));
    cumulativeTurnDeg += Math.toDegrees(omega * dt);
    if (trace != null) {
      trace.add(Vec2.of(x, y));
    }
    if (headingTrace != null) {
      headingTrace.add(startHeadingDeg + cumulativeTurnDeg);
    }
    steps++;
    elapsedMs += timeStepMs;
  }

  @Override
  public long elapsedMs() {
    return elapsedMs;
  }

  @Override
  public int timeStepMs() {
    return timeStepMs;
  }

  @Override
  public void drive(double left, double right) {
    commandedLeft = left;
    commandedRight = right;
    if (firstMoveStep < 0 && (left != 0 || right != 0)) {
      firstMoveStep = steps;
    }
  }

  /** The last wheel command: {left, right} in rad/s. */
  double[] lastCommand() {
    return new double[] {commandedLeft, commandedRight};
  }

  /** True if the last wheel command was "stop". */
  boolean isStopped() {
    return commandedLeft == 0 && commandedRight == 0;
  }

  /** Makes walls and immovable boxes solid: the robot centre can't come within radiusCm. */
  KinematicRobot withStaticObstacles(WorldFile world, double radiusCm) {
    for (WorldFile.Obstacle o : world.staticObstacles()) {
      solids.add(o.corners);
    }
    blockBySolids(radiusCm);
    return this;
  }

  /** Adds a solid square box (e.g. one that has ended up somewhere unexpected). */
  KinematicRobot withBox(Vec2 center, double halfSizeCm) {
    solids.add(new Vec2[] {
        Vec2.of(center.x - halfSizeCm, center.y - halfSizeCm),
        Vec2.of(center.x + halfSizeCm, center.y - halfSizeCm),
        Vec2.of(center.x + halfSizeCm, center.y + halfSizeCm),
        Vec2.of(center.x - halfSizeCm, center.y + halfSizeCm)});
    blockBySolids(ROBOT_RADIUS_CM);
    return this;
  }

  private void blockBySolids(double radiusCm) {
    blocked = p -> {
      for (Vec2[] polygon : solids) {
        if (RoutePlanner.distanceToPolygon(p, polygon) < radiusCm) {
          return true;
        }
      }
      return false;
    };
  }

  /**
   * Front sonars so3 and so4 from the Pioneer3dx proto: position (cm forward, cm left of the
   * centre) and beam direction relative to the heading.
   */
  private static final double[][] FRONT_SONARS = {{16.4, 2.85, 10}, {16.4, -2.64, -10}};
  static final double SONAR_RANGE_CM = 500;
  /** Webots sonars get no echo from a surface hit more than 22.5 degrees off its normal. */
  static final double SONAR_MAX_INCIDENCE_DEG = 22.5;

  /** A single ray per sonar, as in Webots, against the solid obstacles. */
  @Override
  public double frontObstacleCm() {
    double nearest = Double.POSITIVE_INFINITY;
    double h = Math.toRadians(heading);
    double minCos = Math.cos(Math.toRadians(SONAR_MAX_INCIDENCE_DEG));
    for (double[] sonar : FRONT_SONARS) {
      double ox = x + sonar[0] * Math.cos(h) - sonar[1] * Math.sin(h);
      double oy = y + sonar[0] * Math.sin(h) + sonar[1] * Math.cos(h);
      double a = h + Math.toRadians(sonar[2]);
      double dx = Math.cos(a);
      double dy = Math.sin(a);
      for (Vec2[] polygon : solids) {
        for (int i = 0; i < polygon.length; i++) {
          Vec2 p = polygon[i];
          Vec2 q = polygon[(i + 1) % polygon.length];
          double ex = q.x - p.x;
          double ey = q.y - p.y;
          double denom = dx * ey - dy * ex;
          if (Math.abs(denom) < 1e-12) {
            continue; // beam parallel to the edge
          }
          double t = ((p.x - ox) * ey - (p.y - oy) * ex) / denom;
          double u = ((p.x - ox) * dy - (p.y - oy) * dx) / denom;
          if (t < 0 || t > SONAR_RANGE_CM || u < 0 || u > 1) {
            continue;
          }
          double cosIncidence = Math.abs(-dx * ey + dy * ex) / Math.hypot(ex, ey);
          if (cosIncidence >= minCos) {
            nearest = Math.min(nearest, t);
          }
        }
      }
    }
    return nearest;
  }

  @Override
  public double headingDeg() {
    return heading;
  }

  @Override
  public Vec2 positionCm() {
    return Vec2.of(x, y);
  }

  @Override
  public boolean jarTouched() {
    return touchedJar() != null;
  }

  private Vec2 touchedJar() {
    Vec2 here = positionCm();
    for (Vec2 jar : jars) {
      double rel = Angles.delta(heading, here.bearingTo(jar));
      if (here.distanceTo(jar) <= TOUCH_DISTANCE_CM && Math.abs(rel) < 15) {
        return jar;
      }
    }
    return null;
  }

  @Override
  public RgbImage captureImage() {
    int[] px = new int[IMAGE_SIZE * IMAGE_SIZE];
    java.util.Arrays.fill(px, FLOOR);
    Vec2 here = positionCm();
    double halfFov = Math.toRadians(FOV_DEG / 2);
    double pxPerRad = (IMAGE_SIZE / 2.0) / halfFov;
    for (Vec2 jar : jars) {
      double d = Math.max(here.distanceTo(jar), 1);
      double rel = Math.toRadians(Angles.delta(heading, here.bearingTo(jar)));
      if (Math.abs(rel) > halfFov + Math.atan(JAR_RADIUS_CM / d)) {
        continue;
      }
      // Positive rel = jar to the left = smaller column index.
      double cx = IMAGE_SIZE / 2.0 - Math.tan(rel) / Math.tan(halfFov) * IMAGE_SIZE / 2.0;
      double halfWidth = Math.atan(JAR_RADIUS_CM / d) * pxPerRad;
      double halfHeight = Math.atan(JAR_HEIGHT_CM / 2 / d) * pxPerRad;
      int x0 = (int) Math.max(0, Math.round(cx - halfWidth));
      int x1 = (int) Math.min(IMAGE_SIZE - 1, Math.round(cx + halfWidth));
      int y0 = (int) Math.max(0, Math.round(IMAGE_SIZE / 2.0 - halfHeight));
      int y1 = (int) Math.min(IMAGE_SIZE - 1, Math.round(IMAGE_SIZE / 2.0 + halfHeight));
      for (int row = y0; row <= y1; row++) {
        for (int col = x0; col <= x1; col++) {
          px[row * IMAGE_SIZE + col] = JAR;
        }
      }
    }
    return new RgbImage(IMAGE_SIZE, IMAGE_SIZE, px);
  }

  @Override
  public void setFingers(double position) {
    fingers = position;
    if (position >= Config.FINGERS_OPEN - GRIPPER_TOLERANCE) {
      missedThisGrab = false;
    }
    if (position >= Config.FINGERS_OPEN - GRIPPER_TOLERANCE && heldJar != null) {
      // Release in front of the robot.
      double h = Math.toRadians(heading);
      deliveredJars.add(Vec2.of(x + TOUCH_DISTANCE_CM * Math.cos(h), y + TOUCH_DISTANCE_CM * Math.sin(h)));
      deliveryTimesMs.add(elapsedMs);
      heldJar = null;
    }
  }

  @Override
  public void setLift(double position) {
    lift = position;
    if (position <= Config.LIFT_UP + GRIPPER_TOLERANCE
        && fingers <= Config.FINGERS_CLOSED + GRIPPER_TOLERANCE && heldJar == null
        && !missedThisGrab) {
      Vec2 jar = touchedJar();
      if (jar != null && grabsThatMiss > 0) {
        grabsThatMiss--;
        missedThisGrab = true;
        return;
      }
      heldJar = jar;
      jars.remove(heldJar);
    }
  }

  /** Closed fingers squeeze at their limit if they hold a jar; otherwise they don't push. */
  @Override
  public double gripEffort() {
    return heldJar != null && fingers <= Config.FINGERS_CLOSED + GRIPPER_TOLERANCE ? 1.0 : 0.0;
  }

  @Override
  public String diagnostics() {
    return "kinematic model";
  }
}
