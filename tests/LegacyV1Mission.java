import java.util.List;
import java.util.Random;

/**
 * Line-for-line port of v1's whole mission: {@code main} (lines 63-153) and every method it calls,
 * rewritten against {@link RobotPlatform} so v1 can run on the same kinematic model as v2. Like
 * {@link LegacyV1}, it is only used by tests and the benchmark; the original file is unchanged.
 *
 * <p>Differences from the original, none of which change what the robot does:
 * <ul>
 *   <li>Prints are replaced by {@link #activity}, a note of what the robot is doing, so a run
 *       that never ends can say where it got stuck.</li>
 *   <li>{@code Math.random()} (line 437) is a seeded {@link Random}, so runs are repeatable.</li>
 *   <li>v1's loops have no limits of their own. A run ends when the model's step limit throws
 *       {@link RobotPlatform.SimulationEnded}, which is how Webots ends a controller too.</li>
 * </ul>
 */
final class LegacyV1Mission {

  /** How a run went. */
  static final class Result {
    final int delivered;
    final long elapsedMs;
    final boolean finished;
    /** What the robot was doing when the run ended. */
    final String activity;
    /** When it started doing that: for a run that never ends, when it got stuck. */
    final long activitySinceMs;

    Result(int delivered, long elapsedMs, boolean finished, String activity, long activitySinceMs) {
      this.delivered = delivered;
      this.elapsedMs = elapsedMs;
      this.finished = finished;
      this.activity = activity;
      this.activitySinceMs = activitySinceMs;
    }
  }

  private static final double MAX_SPEED = LegacyV1.MAX_SPEED; // :19
  private static final int EPSILON = 10; // :22

  private final RobotPlatform robot;
  private final Random random;
  private int jarsToGo = 5; // :24
  private String activity = "start-up";
  private long activitySinceMs;

  LegacyV1Mission(RobotPlatform robot, long seed) {
    this.robot = robot;
    this.random = new Random(seed);
  }

  /** Runs the mission until it finishes or the simulation ends. */
  Result run() {
    try {
      main();
      return new Result(5 - jarsToGo, robot.elapsedMs(), true, "finished", robot.elapsedMs());
    } catch (RobotPlatform.SimulationEnded e) {
      return new Result(5 - jarsToGo, robot.elapsedMs(), false, activity, activitySinceMs);
    }
  }

  private void doing(String what) {
    activity = what;
    activitySinceMs = robot.elapsedMs();
  }

  /** main, lines 105-153 (after device set-up). */
  private void main() {
    openCloseGripper(0.099f);
    liftLowerGripper(0.001f);
    delay(1000);
    for (int i = 0; i < 10; i++) { // sensor debug test, lines 112-118
      robot.step();
    }
    executeHardcodedPath("hardcodedPathPushing", LegacyV1.clearingPath());

    while (jarsToGo > 0) {
      if (jarsToGo == 5) {
        executeHardcodedPath("navigateToSearchArea", LegacyV1.navigateToSearchArea());
      } else {
        executeHardcodedPath("navigateFromDropoffToSearch", LegacyV1.navigateFromDropoffToSearch());
      }
      boolean success = intelligentJarSearch();
      if (success) {
        executeHardcodedPath("navigateToDropoff", LegacyV1.navigateToDropoff());
        placeJar();
        jarsToGo--;
      } else {
        moveRandomly();
      }
    }
  }

  /** delay, lines 43-49. */
  private void delay(int milliseconds) {
    int elapsedTime = 0;
    while (elapsedTime < milliseconds) {
      robot.step();
      elapsedTime += robot.timeStepMs();
    }
  }

  /** liftLowerGripper, lines 51-54 (the float reaches Webots widened to a double). */
  private void liftLowerGripper(float position) {
    robot.setLift(position);
  }

  /** openCloseGripper, lines 56-61. */
  private void openCloseGripper(float position) {
    robot.setFingers(position);
  }

  /** intelligentJarSearch, lines 156-197. */
  private boolean intelligentJarSearch() {
    int fullRotationAttempts = 0;
    int maxFullRotations = 3;
    while (fullRotationAttempts < maxFullRotations) {
      doing("searching (attempt " + (fullRotationAttempts + 1) + ")");
      Integer bestAngle = LegacyV1.rotateAndFindGreen(robot);
      if (bestAngle == null) {
        moveRandomly();
        fullRotationAttempts++;
        continue;
      }
      turnToAngle(bestAngle);
      boolean success = carefullyApproachGreen();
      if (success) {
        return true;
      }
      backUpDistance(50);
      fullRotationAttempts++;
    }
    return false;
  }

  /** carefullyApproachGreen, lines 273-382. */
  private boolean carefullyApproachGreen() {
    doing("approaching a jar");
    int approachSteps = 0;
    int maxApproachSteps = 400;
    int stepsWithoutGreen = 0;
    int maxStepsWithoutGreen = 30;
    double approachSpeed = MAX_SPEED / 8;

    while (approachSteps < maxApproachSteps) {
      robot.step(); // the loop condition's robot.step(timeStep) != -1
      approachSteps++;
      if (robot.jarTouched()) {
        robot.stop();
        return executeGrab();
      }

      int[][] colorCount = LegacyV1.countColor(robot.captureImage());
      int totalGreen = colorCount[0][0] + colorCount[0][1] + colorCount[0][2];
      if (totalGreen < 15) {
        stepsWithoutGreen++;
        if (stepsWithoutGreen > maxStepsWithoutGreen) {
          robot.stop();
          return false;
        }
      } else {
        stepsWithoutGreen = 0;
      }

      boolean greenLeft = colorCount[0][0] > colorCount[0][1] + EPSILON
          && colorCount[0][0] > colorCount[0][2] + EPSILON;
      boolean greenCenter = colorCount[0][1] > colorCount[0][0] + EPSILON
          && colorCount[0][1] > colorCount[0][2] + EPSILON;
      boolean greenRight = colorCount[0][2] > colorCount[0][0] + EPSILON
          && colorCount[0][2] > colorCount[0][1] + EPSILON;

      if (greenLeft) {
        robot.drive(approachSpeed * 0.3, approachSpeed * 1.2);
      } else if (greenRight) {
        robot.drive(approachSpeed * 1.2, approachSpeed * 0.3);
      } else if (greenCenter || totalGreen > 30) {
        robot.drive(approachSpeed, approachSpeed);
      } else {
        robot.stop();
        for (int i = 0; i < 5; i++) {
          robot.step();
        }
      }

      if (approachSteps > 300) {
        robot.stop();
        return false;
      }
    }
    robot.stop();
    return false;
  }

  /** executeGrab, lines 385-430. */
  private boolean executeGrab() {
    if (!robot.jarTouched()) {
      return false;
    }
    robot.stop();
    for (int i = 0; i < 10; i++) {
      robot.step();
    }
    for (int i = 0; i < 100; i++) {
      robot.step();
      openCloseGripper(0.01f);
      robot.stop();
    }
    for (int i = 0; i < 120; i++) {
      robot.step();
      liftLowerGripper(-0.0499f);
      openCloseGripper(0.01f);
      robot.stop();
    }
    for (int i = 0; i < 20; i++) {
      robot.step();
    }
    return true;
  }

  /** moveRandomly, lines 433-450. */
  private void moveRandomly() {
    doing("moving to a random spot");
    int randomTurn = (int) (random.nextDouble() * 180) - 90;
    rotateByDegrees(randomTurn);
    robot.drive(MAX_SPEED / 3, MAX_SPEED / 3);
    for (int i = 0; i < 60; i++) {
      robot.step();
    }
    robot.stop();
  }

  /** backUpDistance, lines 453-464. */
  private void backUpDistance(int steps) {
    for (int i = 0; i < steps; i++) {
      robot.step();
      robot.drive(-MAX_SPEED / 3, -MAX_SPEED / 3);
    }
    robot.stop();
  }

  /** rotateByDegrees, lines 467-491. */
  private void rotateByDegrees(int degrees) {
    int startAngle = compass();
    int targetAngle = (startAngle + degrees + 360) % 360;
    if (degrees > 0) {
      robot.drive(-MAX_SPEED / 3, MAX_SPEED / 3);
    } else {
      robot.drive(MAX_SPEED / 3, -MAX_SPEED / 3);
    }
    int timeout = 150;
    while (true) {
      robot.step(); // robot.step(timeStep) != -1 runs before the timeout check
      if (timeout <= 0) {
        break;
      }
      int currentAngle = (compass() + 360) % 360;
      int diff = Math.abs(currentAngle - targetAngle);
      if (diff < 5 || diff > 355) {
        break;
      }
      timeout--;
    }
    robot.stop();
  }

  /** turnToAngle, lines 494-502. */
  private void turnToAngle(int targetAngle) {
    int currentAngle = compass();
    int diff = (targetAngle - currentAngle + 540) % 360 - 180;
    rotateByDegrees(diff);
  }

  /** placeJar, lines 627-653. */
  private void placeJar() {
    executeHardcodedPath("placeJar", LegacyV1.placeJarPath(jarsToGo));
    doing("releasing the jar");
    for (int i = 0; i < 50; i++) {
      robot.step();
      openCloseGripper(0.099f);
      liftLowerGripper(0.001f);
    }
    backUpDistance(50);
  }

  /**
   * executeHardcodedPath, lines 684-697: each leg starts from wherever the robot is, turns towards
   * the waypoint and drives the leg's length (makeHardcodedTurn and moveHardcodedAhead, ported in
   * {@link LegacyV1} with no step limit of their own).
   */
  private void executeHardcodedPath(String name, List<Vec2> waypoints) {
    for (int i = 0; i < waypoints.size(); i++) {
      Vec2 here = robot.positionCm();
      Vec2 target = waypoints.get(i);
      String leg = " " + target + " (leg " + (i + 1) + " of " + waypoints.size() + " of `" + name
          + "`)";
      doing("turning towards" + leg);
      LegacyV1.makeHardcodedTurn(robot, here.x, here.y, target.x, target.y, Integer.MAX_VALUE);
      doing("driving to" + leg);
      LegacyV1.moveHardcodedAhead(robot, here.x, here.y, target.x, target.y, Integer.MAX_VALUE);
    }
  }

  private int compass() {
    return LegacyV1.compassDegrees(robot.headingDeg());
  }
}
