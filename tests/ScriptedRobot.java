import java.util.function.IntPredicate;

/**
 * A tiny scripted {@link RobotPlatform} for exact edge cases the kinematic model can't stage:
 * the camera shows a fixed (or per-step) number of green pixels per third of the image, the
 * bumper is pressed on chosen steps, the heading turns at a fixed rate whenever the wheels turn,
 * and the position can orbit a circle regardless of the wheel commands.
 */
final class ScriptedRobot implements RobotPlatform {
  int left;
  int center;
  int right;
  /** When set, overrides left/center/right: maps the step count to {left, center, right}. */
  java.util.function.IntFunction<int[]> sectorsAtStep;
  IntPredicate touchedAtStep = step -> false;
  double turnDegPerStep = 1.5;
  double orbitRadiusCm;
  double orbitDegPerStep;
  int steps;
  int captures;
  /** Every wheel command, in order. */
  final java.util.List<double[]> commands = new java.util.ArrayList<>();
  /** Whether the wheels were commanded to move during each step (index = step number - 1). */
  final java.util.List<Boolean> movingDuring = new java.util.ArrayList<>();
  double heading;
  double leftSpeed;
  double rightSpeed;
  double fingers = Config.FINGERS_OPEN;
  double lift = Config.LIFT_DOWN;

  @Override
  public void step() {
    movingDuring.add(!isStopped());
    steps++;
    if (rightSpeed != leftSpeed) {
      heading = Angles.normalize(heading + Math.signum(rightSpeed - leftSpeed) * turnDegPerStep);
    }
  }

  @Override
  public long elapsedMs() {
    return steps * 32L;
  }

  @Override
  public int timeStepMs() {
    return 32;
  }

  @Override
  public void drive(double l, double r) {
    leftSpeed = l;
    rightSpeed = r;
    commands.add(new double[] {l, r});
  }

  boolean isStopped() {
    return leftSpeed == 0 && rightSpeed == 0;
  }

  @Override
  public double headingDeg() {
    return heading;
  }

  @Override
  public Vec2 positionCm() {
    double a = Math.toRadians(steps * orbitDegPerStep);
    return Vec2.of(orbitRadiusCm * Math.sin(a), orbitRadiusCm - orbitRadiusCm * Math.cos(a));
  }

  @Override
  public boolean jarTouched() {
    return touchedAtStep.test(steps);
  }

  /** Green pixels: {@code left}, {@code center}, {@code right} in each third (21 per row). */
  @Override
  public RgbImage captureImage() {
    captures++;
    int[] px = new int[64 * 64];
    java.util.Arrays.fill(px, RgbImage.pack(190, 150, 110));
    int[] sectors = sectorsAtStep == null ? new int[] {left, center, right}
        : sectorsAtStep.apply(steps);
    paint(px, 0, sectors[0]);
    paint(px, 21, sectors[1]);
    paint(px, 43, sectors[2]);
    return new RgbImage(64, 64, px);
  }

  private static void paint(int[] px, int firstColumn, int count) {
    for (int i = 0; i < count; i++) {
      px[(i / 21) * 64 + firstColumn + (i % 21)] = RgbImage.pack(30, 160, 30);
    }
  }

  @Override
  public void setFingers(double position) {
    fingers = position;
  }

  @Override
  public void setLift(double position) {
    lift = position;
  }
}
