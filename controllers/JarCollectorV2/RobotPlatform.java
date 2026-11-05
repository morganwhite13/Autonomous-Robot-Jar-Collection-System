/**
 * Everything the mission logic needs from the robot.
 *
 * <p>{@link WebotsPlatform} implements it on top of the Webots API. Tests implement it with a
 * kinematic model, which is what lets the navigation and search code be tested without Webots.
 */
interface RobotPlatform {

  /** Thrown by {@link #step()} when Webots ends the simulation, so every loop unwinds cleanly. */
  final class SimulationEnded extends RuntimeException {
    private static final long serialVersionUID = 1L;

    SimulationEnded() {
      super("Webots ended the simulation");
    }
  }

  /** Advances the simulation by one control step. */
  void step();

  /** Simulated time since the controller started, in milliseconds. */
  long elapsedMs();

  int timeStepMs();

  /** Wheel angular velocities in rad/s; positive is forwards. */
  void drive(double leftSpeed, double rightSpeed);

  default void stop() {
    drive(0, 0);
  }

  default void waitSteps(int steps) {
    for (int i = 0; i < steps; i++) {
      step();
    }
  }

  /** Compass heading in degrees, [0, 360). */
  double headingDeg();

  /** Ground-truth position from the Supervisor API, in cm. */
  Vec2 positionCm();

  /** True while the front bumper (touch sensor) is pressed. */
  boolean jarTouched();

  RgbImage captureImage();

  void setFingers(double position);

  void setLift(double position);

  /** One-line description of auxiliary sensors for the start-up self-test. */
  default String diagnostics() {
    return "";
  }
}
