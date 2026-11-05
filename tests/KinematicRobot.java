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
  private long elapsedMs;
  private long steps;
  private long maxSteps = 500_000;
  /** Signed total rotation since construction (positive = counter-clockwise). */
  double cumulativeTurnDeg;

  final List<Vec2> jars = new ArrayList<>();
  final List<Vec2> deliveredJars = new ArrayList<>();
  Vec2 heldJar;
  double fingers = Config.FINGERS_OPEN;
  double lift = Config.LIFT_DOWN;
  /** Positions the robot centre may not enter (e.g. a wall for stall tests). */
  Predicate<Vec2> blocked = p -> false;

  KinematicRobot(Vec2 position, double headingDeg) {
    this(position, headingDeg, 32);
  }

  KinematicRobot(Vec2 position, double headingDeg, int timeStepMs) {
    this.x = position.x;
    this.y = position.y;
    this.heading = Angles.normalize(headingDeg);
    this.timeStepMs = timeStepMs;
  }

  /** Ends the "simulation" after this many steps so a buggy loop fails the test instead of hanging. */
  KinematicRobot withStepLimit(long limit) {
    this.maxSteps = limit;
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
    double dt = timeStepMs / 1000.0;
    double v = WHEEL_RADIUS_CM * (leftSpeed + rightSpeed) / 2;
    double omega = WHEEL_RADIUS_CM * (rightSpeed - leftSpeed) / AXLE_TRACK_CM;
    double mid = Math.toRadians(heading) + omega * dt / 2;
    Vec2 next = Vec2.of(x + v * dt * Math.cos(mid), y + v * dt * Math.sin(mid));
    if (!blocked.test(next)) {
      x = next.x;
      y = next.y;
    }
    heading = Angles.normalize(heading + Math.toDegrees(omega * dt));
    cumulativeTurnDeg += Math.toDegrees(omega * dt);
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
    leftSpeed = left;
    rightSpeed = right;
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
    if (position >= Config.FINGERS_OPEN && heldJar != null) {
      // Release in front of the robot.
      double h = Math.toRadians(heading);
      deliveredJars.add(Vec2.of(x + TOUCH_DISTANCE_CM * Math.cos(h), y + TOUCH_DISTANCE_CM * Math.sin(h)));
      heldJar = null;
    }
  }

  @Override
  public void setLift(double position) {
    lift = position;
    if (position <= Config.LIFT_UP && fingers <= Config.FINGERS_CLOSED && heldJar == null) {
      heldJar = touchedJar();
      jars.remove(heldJar);
    }
  }

  @Override
  public String diagnostics() {
    return "kinematic model";
  }
}
