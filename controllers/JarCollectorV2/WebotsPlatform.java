import com.cyberbotics.webots.controller.Camera;
import com.cyberbotics.webots.controller.Compass;
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.Field;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Supervisor;
import com.cyberbotics.webots.controller.TouchSensor;
import java.util.Locale;

/**
 * {@link RobotPlatform} backed by Webots. The only class (with the entry point) that imports the
 * Webots API. Device names and set-up are identical to v1.
 */
final class WebotsPlatform implements RobotPlatform {

  private final Supervisor robot;
  private final int timeStep;
  private final Field translation;
  private final Motor leftWheel;
  private final Motor rightWheel;
  private final Motor lift;
  private final Motor leftFinger;
  private final Motor rightFinger;
  private final DistanceSensor leftAhead;
  private final DistanceSensor rightAhead;
  private final DistanceSensor leftAngled;
  private final DistanceSensor rightAngled;
  private final DistanceSensor leftSide;
  private final DistanceSensor rightSide;
  private final TouchSensor bumper;
  private final Compass compass;
  private final Camera camera;
  private long elapsedMs;

  WebotsPlatform() {
    robot = new Supervisor();
    timeStep = (int) Math.round(robot.getBasicTimeStep());
    translation = robot.getSelf().getField("translation");

    leftWheel = robot.getMotor("left wheel");
    rightWheel = robot.getMotor("right wheel");
    leftWheel.setPosition(Double.POSITIVE_INFINITY);
    rightWheel.setPosition(Double.POSITIVE_INFINITY);
    leftWheel.setVelocity(0);
    rightWheel.setVelocity(0);

    lift = robot.getMotor("lift motor");
    leftFinger = robot.getMotor("left finger motor");
    rightFinger = robot.getMotor("right finger motor");
    leftFinger.enableForceFeedback(timeStep);
    rightFinger.enableForceFeedback(timeStep);

    leftSide = enable(robot.getDistanceSensor("so0"));
    leftAngled = enable(robot.getDistanceSensor("so1"));
    leftAhead = enable(robot.getDistanceSensor("so3"));
    rightAhead = enable(robot.getDistanceSensor("so4"));
    rightAngled = enable(robot.getDistanceSensor("so6"));
    rightSide = enable(robot.getDistanceSensor("so7"));

    camera = robot.getCamera("camera");
    camera.enable(timeStep);
    compass = robot.getCompass("compass");
    compass.enable(timeStep);
    bumper = robot.getTouchSensor("touch sensor");
    bumper.enable(timeStep);
  }

  private DistanceSensor enable(DistanceSensor sensor) {
    sensor.enable(timeStep);
    return sensor;
  }

  @Override
  public void step() {
    if (robot.step(timeStep) == -1) {
      throw new SimulationEnded();
    }
    elapsedMs += timeStep;
  }

  @Override
  public long elapsedMs() {
    return elapsedMs;
  }

  @Override
  public int timeStepMs() {
    return timeStep;
  }

  @Override
  public void drive(double leftSpeed, double rightSpeed) {
    leftWheel.setVelocity(leftSpeed);
    rightWheel.setVelocity(rightSpeed);
  }

  @Override
  public double headingDeg() {
    return Angles.fromCompass(compass.getValues());
  }

  @Override
  public Vec2 positionCm() {
    return Vec2.fromWorld(translation.getSFVec3f());
  }

  @Override
  public boolean jarTouched() {
    return bumper.getValue() > 0.5; // bumper-type touch sensors report exactly 0 or 1
  }

  @Override
  public RgbImage captureImage() {
    int width = camera.getWidth();
    int height = camera.getHeight();
    int[] raw = camera.getImage();
    int[] rgb = new int[width * height];
    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        rgb[y * width + x] = RgbImage.pack(
            Camera.imageGetRed(raw, width, x, y),
            Camera.imageGetGreen(raw, width, x, y),
            Camera.imageGetBlue(raw, width, x, y));
      }
    }
    return new RgbImage(width, height, rgb);
  }

  @Override
  public void setFingers(double position) {
    leftFinger.setVelocity(Config.GRIPPER_SPEED);
    rightFinger.setVelocity(Config.GRIPPER_SPEED);
    leftFinger.setPosition(position);
    rightFinger.setPosition(position);
  }

  @Override
  public void setLift(double position) {
    lift.setVelocity(Config.GRIPPER_SPEED);
    lift.setPosition(position);
  }

  @Override
  public double gripEffort() {
    double max = Math.min(leftFinger.getMaxForce(), rightFinger.getMaxForce());
    if (!(max > 0)) {
      return Double.NaN;
    }
    // Each finger pushes inwards; the sign depends on its slider axis, so use magnitudes.
    return (Math.abs(leftFinger.getForceFeedback()) + Math.abs(rightFinger.getForceFeedback()))
        / (2 * max);
  }

  @Override
  public String diagnostics() {
    return String.format(Locale.ROOT,
        "sonar front L=%.0f R=%.0f, angled L=%.0f R=%.0f, side L=%.0f R=%.0f",
        leftAhead.getValue(), rightAhead.getValue(), leftAngled.getValue(),
        rightAngled.getValue(), leftSide.getValue(), rightSide.getValue());
  }
}
