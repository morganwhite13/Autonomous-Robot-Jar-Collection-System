import java.util.ArrayList;
import java.util.List;

/**
 * Line-for-line ports of algorithms from the original controller
 * (controllers/ProjectController3/ProjectController3.java), rewritten against
 * {@link RobotPlatform} so the "before" behaviour can be measured on the same kinematic model as
 * v2. Only used by tests and the benchmark; the original file itself is never modified.
 */
final class LegacyV1 {
  private LegacyV1() {}

  static final double MAX_SPEED = 5; // :19
  private static final int BFFR = 35; // :664

  // ------------------------------------------------------------------ routes, original expressions

  /** hardcodedPathPushing, lines 664-680 (without the leading "current position" point). */
  static List<Vec2> clearingPath() {
    int bffr = BFFR;
    return pts(
        -980 + 60 + bffr, 530,
        -970 + 60 + bffr, 800,
        -814, 555,
        -814, 190 + 30 + bffr + 120,
        -441 - 30 - 60 - bffr, 190 + 30 + bffr + 120,
        -441 - 30 - bffr, 800,
        -145 - 30 - bffr, 447 + 50,
        -145 - 30 - bffr, 122,
        -145 - 30 - bffr, 278,
        565 - 150 - 60 - bffr, 278,
        565 - 150 - 60 - bffr, 0 + 60 + bffr,
        543 - 30 - bffr, 144,
        991 - 20 - 60 - bffr, 144,
        875, 100,
        850, 60);
  }

  /** navigateToSearchArea, lines 572-573. */
  static List<Vec2> navigateToSearchArea() {
    return pts(850, 100, 850, 60);
  }

  /** navigateToDropoff, lines 614-623. */
  static List<Vec2> navigateToDropoff() {
    return pts(850, 100, 350, 144, 0, 144, -75, 278, -215, 278, -340, 605, -590, 510, -590, 390,
        -815, 410, -850, 602);
  }

  /** navigateFromDropoffToSearch, lines 588-599. */
  static List<Vec2> navigateFromDropoffToSearch() {
    return pts(-850, 602, -815, 410, -590, 390, -590, 510, -340, 605, -220, 400, -215, 278,
        -75, 278, 0, 144, 350, 144, 850, 100, 850, 60);
  }

  /** placeJar, lines 636-641. {@code jarsToGo} is the value before the jar is counted. */
  static List<Vec2> placeJarPath(int jarsToGo) {
    int yVal = 602 + (5 - jarsToGo) * 30 + 5;
    return pts(-850, yVal, -900, yVal, -915, yVal, -936, yVal);
  }

  private static List<Vec2> pts(int... xy) {
    List<Vec2> out = new ArrayList<>();
    for (int i = 0; i < xy.length; i += 2) {
      out.add(Vec2.of(xy[i], xy[i + 1]));
    }
    return out;
  }

  // ------------------------------------------------------------------ sensing

  /** getCompassReadingInDegrees, lines 514-523, verbatim. */
  static int compassDegrees(double[] compassReadings) {
    double rad = Math.atan2(compassReadings[0], compassReadings[2]);
    double bearing = -((rad + Math.PI) / Math.PI * 180.0);
    if (bearing > 180) {
      bearing = 360 - bearing;
    }
    if (bearing < -180) {
      bearing = 360 + bearing;
    }
    return (int) (bearing);
  }

  /** What v1's compass function returns for a physical heading in [0, 360). */
  static int compassDegrees(double headingDeg) {
    return (int) (headingDeg >= 180 ? headingDeg - 360 : headingDeg);
  }

  /** countColor green test, line 543. */
  static boolean isGreen(int r, int g, int b) {
    return r < 120 && g > 35 && b < 120;
  }

  /** countColor, lines 526-559, verbatim (blue counting included). Arrays are [x][y]. */
  static int[][] countColor(int[][] r, int[][] g, int[][] b) {
    int cameraWidth = r.length;
    int heightCaptured = r[0].length;
    int[][] results = new int[2][3];
    int greenMaximum = 120;
    int greenMinimum = 35;
    int blueMaximum = 20;
    int blueMinimum = 80;
    int third = (int) Math.floor(cameraWidth / 3);
    for (int j = 0; j < heightCaptured; j++) {
      for (int i = 0; i < third; i++) {
        if (r[i][j] < greenMaximum && g[i][j] > greenMinimum && b[i][j] < greenMaximum) results[0][0]++;
        if (r[i][j] < blueMaximum && g[i][j] < blueMaximum && b[i][j] > blueMinimum) results[1][0]++;
      }
      for (int i = third; i < cameraWidth - third; i++) {
        if (r[i][j] < greenMaximum && g[i][j] > greenMinimum && b[i][j] < greenMaximum) results[0][1]++;
        if (r[i][j] < blueMaximum && g[i][j] < blueMaximum && b[i][j] > blueMinimum) results[1][1]++;
      }
      for (int i = cameraWidth - third; i < cameraWidth; i++) {
        if (r[i][j] < greenMaximum && g[i][j] > greenMinimum && b[i][j] < greenMaximum) results[0][2]++;
        if (r[i][j] < blueMaximum && g[i][j] < blueMaximum && b[i][j] > blueMinimum) results[1][2]++;
      }
    }
    return results;
  }

  /** v1 copied each frame into three [x][y] arrays before counting (lines 217-228). */
  static int[][] countColor(RgbImage image) {
    int[][] r = new int[image.width][image.height];
    int[][] g = new int[image.width][image.height];
    int[][] b = new int[image.width][image.height];
    for (int x = 0; x < image.width; x++) {
      for (int y = 0; y < image.height; y++) {
        r[x][y] = image.red(x, y);
        g[x][y] = image.green(x, y);
        b[x][y] = image.blue(x, y);
      }
    }
    return countColor(r, g, b);
  }

  // ------------------------------------------------------------------ motion

  /**
   * makeHardcodedTurn, lines 722-765. Returns the number of steps taken, or -1 if it had not
   * converged after {@code maxSteps} (v1 itself had no limit).
   */
  static int makeHardcodedTurn(RobotPlatform robot, double x1, double y1, double x2, double y2,
      int maxSteps) {
    double xDiff = x2 - x1;
    double yDiff = y2 - y1;
    int startAngle = compassDegrees(robot.headingDeg());
    int turn = (int) (Math.atan2(yDiff, xDiff) * 180 / Math.PI);
    turn = (turn - startAngle) % 360;
    if (turn < -180) {
      turn += 360;
    } else if (turn > 180) {
      turn -= 360;
    }
    int finalAngle = (startAngle + turn + 360) % 360;
    if (turn > 0) {
      robot.drive(-MAX_SPEED, MAX_SPEED);
    } else {
      robot.drive(MAX_SPEED, -MAX_SPEED);
    }
    for (int steps = 1; steps <= maxSteps; steps++) {
      robot.step();
      int degrees = (compassDegrees(robot.headingDeg()) + 360) % 360;
      if (degrees > finalAngle - 3 && degrees < finalAngle + 3) {
        robot.stop();
        return steps;
      }
      int distTurningRight = (degrees - finalAngle + 360) % 360;
      int distTurningLeft = (finalAngle - degrees + 360) % 360;
      if (distTurningRight < distTurningLeft) {
        robot.drive(MAX_SPEED, -MAX_SPEED);
      } else {
        robot.drive(-MAX_SPEED, MAX_SPEED);
      }
    }
    robot.stop();
    return -1;
  }

  /**
   * moveHardcodedAhead, lines 699-720: full speed, no steering, stop once the distance from the
   * start equals the leg length. Returns steps taken or -1 if not finished after maxSteps.
   */
  static int moveHardcodedAhead(RobotPlatform robot, double x1, double y1, double x2, double y2,
      int maxSteps) {
    double desiredDistance = Math.hypot(x2 - x1, y2 - y1);
    robot.drive(MAX_SPEED, MAX_SPEED);
    for (int steps = 1; steps <= maxSteps; steps++) {
      robot.step();
      Vec2 p = robot.positionCm();
      if (Math.hypot(p.x - x1, p.y - y1) >= desiredDistance) {
        robot.stop();
        return steps;
      }
    }
    robot.stop();
    return -1;
  }

  /**
   * rotateAndFindGreen, lines 200-270: rotate for a fixed 200 steps, sampling every 5th.
   * Returns the heading of the best detection (as a v1 int), or null if nothing was seen.
   */
  static Integer rotateAndFindGreen(RobotPlatform robot) {
    Integer bestAngle = null;
    int bestScore = 0;
    robot.drive(-MAX_SPEED / 4, MAX_SPEED / 4);
    for (int step = 0; step < 200; step++) {
      robot.step();
      if (step % 5 == 0) {
        int[][] colorCount = countColor(robot.captureImage());
        int totalGreen = colorCount[0][0] + colorCount[0][1] + colorCount[0][2];
        if (totalGreen > 20) {
          int currentAngle = compassDegrees(robot.headingDeg());
          String section = "center";
          if (colorCount[0][0] > colorCount[0][1] && colorCount[0][0] > colorCount[0][2]) {
            section = "left";
          } else if (colorCount[0][2] > colorCount[0][1] && colorCount[0][2] > colorCount[0][0]) {
            section = "right";
          }
          int score = totalGreen + (section.equals("center") ? 50 : 0);
          if (score > bestScore) {
            bestScore = score;
            bestAngle = currentAngle;
          }
        }
      }
    }
    robot.stop();
    return bestAngle;
  }

  /** Effective approach limit: the 400-step loop (line 277) always lost to the 300-step check (line 370). */
  static final int APPROACH_EFFECTIVE_STEPS = 300;
  static final double APPROACH_SPEED = MAX_SPEED / 8; // line 281
}
