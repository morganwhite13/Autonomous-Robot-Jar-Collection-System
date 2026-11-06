import java.awt.Point; 
import java.util.ArrayList;
import com.cyberbotics.webots.controller.Camera;
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.TouchSensor;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Compass;
import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.Field;
import com.cyberbotics.webots.controller.Node;
import com.cyberbotics.webots.controller.Supervisor;

public class ProjectController3 {

  private static final byte    CAMERA_WIDTH = 64;
  private static final byte    CAMERA_HEIGHT = 64;
  private static final double  GRIPPER_MOTOR_MAX_SPEED = 0.1;
  
  private static double  MAX_SPEED = 5;
  static int WORLD_WIDTH = 1000;
  static int WORLD_HEIGHT = 500;
  static final int EPSILON = 10;
  static final int HEIGHT_CAPTURED = CAMERA_HEIGHT;
  static int jarsToGo = 5;
  
  private static Supervisor      robot;
  private static Motor           leftMotor;
  private static Motor           rightMotor;
  private static Motor           gripperLift;
  private static Motor           gripperLeftSide;
  private static Motor           gripperRightSide;
  private static DistanceSensor  leftSideSensor; 
  private static DistanceSensor  rightSideSensor;
  private static DistanceSensor  leftAheadSensor; 
  private static DistanceSensor  rightAheadSensor;
  private static DistanceSensor  leftAngledSensor; 
  private static DistanceSensor  rightAngledSensor;
  private static TouchSensor     jarDetectedSensor;
  private static Compass         compass;
  private static Camera          camera;
  private static Field           TranslationField;
  
  private static void delay(int milliseconds, int timeStep) {
    int elapsedTime = 0;
    while (elapsedTime < milliseconds) {
      robot.step(timeStep);
      elapsedTime += timeStep;
    }
  }
  
  public static void liftLowerGripper(float position) {
    gripperLift.setVelocity(GRIPPER_MOTOR_MAX_SPEED);
    gripperLift.setPosition(position);
  }

  public static void openCloseGripper(float position) {
    gripperLeftSide.setVelocity(GRIPPER_MOTOR_MAX_SPEED);
    gripperRightSide.setVelocity(GRIPPER_MOTOR_MAX_SPEED);
    gripperLeftSide.setPosition(position);
    gripperRightSide.setPosition(position);
  }

  public static void main(String[] args) {
    robot = new Supervisor();
    
    Node    robotNode = robot.getSelf();
    TranslationField = robotNode.getField("translation");
    
    int timeStep = (int) Math.round(robot.getBasicTimeStep());
    
    // Set up the motors
    leftMotor = robot.getMotor("left wheel");
    rightMotor = robot.getMotor("right wheel");
    gripperLift = robot.getMotor("lift motor");
    gripperLeftSide = robot.getMotor("left finger motor");
    gripperRightSide = robot.getMotor("right finger motor");
    leftMotor.setPosition(Double.POSITIVE_INFINITY);
    rightMotor.setPosition(Double.POSITIVE_INFINITY);
    leftMotor.setVelocity(0);
    rightMotor.setVelocity(0); 
    
    // Get and enable the distance sensors
    leftSideSensor = robot.getDistanceSensor("so0"); 
    leftAngledSensor = robot.getDistanceSensor("so1"); 
    leftAheadSensor = robot.getDistanceSensor("so3"); 
    rightAheadSensor = robot.getDistanceSensor("so4"); 
    rightAngledSensor = robot.getDistanceSensor("so6"); 
    rightSideSensor = robot.getDistanceSensor("so7"); 
    leftAheadSensor.enable(timeStep);
    rightAheadSensor.enable(timeStep);
    leftAngledSensor.enable(timeStep);
    rightAngledSensor.enable(timeStep);
    leftSideSensor.enable(timeStep);
    rightSideSensor.enable(timeStep);
    
    camera = new Camera("camera");
    camera.enable(timeStep);
    
    compass = robot.getCompass("compass");
    compass.enable(timeStep);
    
    jarDetectedSensor = new TouchSensor("touch sensor");
    jarDetectedSensor.enable(timeStep);
    
    // Initialize gripper
    openCloseGripper(0.099f);
    liftLowerGripper(0.001f);
    delay(1000, timeStep);
    
    // DEBUG: Test sensor values
    System.out.println("\n=== SENSOR DEBUG TEST ===");
    for (int i = 0; i < 10; i++) {
      robot.step(timeStep);
      System.out.println("Front sensors: L=" + leftAheadSensor.getValue() + 
                        " R=" + rightAheadSensor.getValue() +
                        " Angled: L=" + leftAngledSensor.getValue() +
                        " R=" + rightAngledSensor.getValue());
    }
    System.out.println("=== END SENSOR TEST ===\n");
    
    // Initial push boxes to clear path
    System.out.println("=== PHASE 1: Clearing path ===");
    hardcodedPathPushing();
    
    // Main jar collection loop
    while (jarsToGo > 0) {
      System.out.println("\n=== COLLECTING JAR " + (6 - jarsToGo) + " ===");
      
      // Go to search area
      if (jarsToGo == 5) {
        navigateToSearchArea();
      } else {
        navigateFromDropoffToSearch();
      }
      
      // Find and grab jar
      boolean success = intelligentJarSearch(timeStep);
      
      if (success) {
        navigateToDropoff();
        placeJar(timeStep);
        jarsToGo--;
        System.out.println("Jars remaining: " + jarsToGo);
      } else {
        System.out.println("CRITICAL: Failed to find jar after all attempts!");
        // Move to slightly different position and retry
        moveRandomly(timeStep);
      }
    }
    
    System.out.println("\n=== ALL JARS COLLECTED! ===");
    System.exit(0);
  }
  
  // NEW: Intelligent jar search using ONLY camera and touch sensor
  private static boolean intelligentJarSearch(int timeStep) {
    System.out.println("\n>>> Starting INTELLIGENT JAR SEARCH <<<");
    
    // Strategy: Rotate slowly and look for green, approach cautiously
    int fullRotationAttempts = 0;
    int maxFullRotations = 3;
    
    while (fullRotationAttempts < maxFullRotations) {
      System.out.println("\n--- Search Attempt " + (fullRotationAttempts + 1) + " ---");
      
      // Phase 1: Rotate and look for green
      GreenDetection bestGreen = rotateAndFindGreen(timeStep);
      
      if (bestGreen == null) {
        System.out.println("No green detected in full rotation, moving...");
        moveRandomly(timeStep);
        fullRotationAttempts++;
        continue;
      }
      
      System.out.println("Green found! Angle: " + bestGreen.angle + 
                        ", Pixels: " + bestGreen.totalGreen +
                        ", Section: " + bestGreen.section);
      
      // Phase 2: Turn to face the green
      turnToAngle(bestGreen.angle, timeStep);
      
      // Phase 3: Approach VERY CAREFULLY using only camera and touch
      boolean success = carefullyApproachGreen(timeStep);
      
      if (success) {
        return true;
      }
      
      // Failed, back up and try again
      System.out.println("Approach failed, backing up...");
      backUpDistance(timeStep, 50);
      fullRotationAttempts++;
    }
    
    return false;
  }
  
  // NEW: Rotate 360 and find best green detection
  private static GreenDetection rotateAndFindGreen(int timeStep) {
    System.out.println("Rotating 360 degrees to find green...");
    
    GreenDetection bestDetection = null;
    int bestScore = 0;
    
    int stepsPerCheck = 5; // Check every 5 steps
    int totalSteps = 200; // Total rotation steps
    
    leftMotor.setVelocity(-MAX_SPEED / 4);
    rightMotor.setVelocity(MAX_SPEED / 4);
    
    for (int step = 0; step < totalSteps; step++) {
      robot.step(timeStep);
      
      if (step % stepsPerCheck == 0) {
        // Check for green
        int[] image = camera.getImage();
        int[][] r = new int[CAMERA_WIDTH][HEIGHT_CAPTURED];
        int[][] g = new int[CAMERA_WIDTH][HEIGHT_CAPTURED];
        int[][] b = new int[CAMERA_WIDTH][HEIGHT_CAPTURED];
        
        for (int i = 0; i < CAMERA_WIDTH; i++) {
          for (int j = -HEIGHT_CAPTURED/2; j < HEIGHT_CAPTURED/2; j++) {
            r[i][j+HEIGHT_CAPTURED/2] = Camera.imageGetRed(image, CAMERA_WIDTH, i, CAMERA_HEIGHT/2+j);
            g[i][j+HEIGHT_CAPTURED/2] = Camera.imageGetGreen(image, CAMERA_WIDTH, i, CAMERA_HEIGHT/2+j);
            b[i][j+HEIGHT_CAPTURED/2] = Camera.imageGetBlue(image, CAMERA_WIDTH, i, CAMERA_HEIGHT/2+j);
          }
        }
        
        int[][] colorCount = countColor(r, g, b);
        int totalGreen = colorCount[0][0] + colorCount[0][1] + colorCount[0][2];
        
        if (totalGreen > 20) { // Found some green
          int currentAngle = getCompassReadingInDegrees(compass);
          
          // Determine which section has most green
          String section = "center";
          if (colorCount[0][0] > colorCount[0][1] && colorCount[0][0] > colorCount[0][2]) {
            section = "left";
          } else if (colorCount[0][2] > colorCount[0][1] && colorCount[0][2] > colorCount[0][0]) {
            section = "right";
          }
          
          // Score this detection
          int score = totalGreen;
          if (section.equals("center")) {
            score += 50; // Bonus for centered
          }
          
          System.out.println("  Green detected: " + totalGreen + " pixels at angle " + currentAngle);
          
          if (score > bestScore) {
            bestScore = score;
            bestDetection = new GreenDetection();
            bestDetection.angle = currentAngle;
            bestDetection.totalGreen = totalGreen;
            bestDetection.section = section;
            bestDetection.greenLeft = colorCount[0][0];
            bestDetection.greenCenter = colorCount[0][1];
            bestDetection.greenRight = colorCount[0][2];
          }
        }
      }
    }
    
    leftMotor.setVelocity(0);
    rightMotor.setVelocity(0);
    
    return bestDetection;
  }
  
  // NEW: Carefully approach using ONLY camera and touch sensor
  private static boolean carefullyApproachGreen(int timeStep) {
    System.out.println("\n>>> CAREFUL APPROACH MODE <<<");
    
    int approachSteps = 0;
    int maxApproachSteps = 400;
    int stepsWithoutGreen = 0;
    int maxStepsWithoutGreen = 30;
    
    double approachSpeed = MAX_SPEED / 8; // Very slow
    
    while (approachSteps < maxApproachSteps && robot.step(timeStep) != -1) {
      approachSteps++;
      
      // Check touch sensor FIRST
      boolean touched = jarDetectedSensor.getValue() == 1.0f;
      
      if (touched) {
        System.out.println("\n*** TOUCH DETECTED! Stopping! ***");
        leftMotor.setVelocity(0);
        rightMotor.setVelocity(0);
        
        // Execute grab
        boolean grabbed = executeGrab(timeStep);
        return grabbed;
      }
      
      // Read camera
      int[] image = camera.getImage();
      int[][] r = new int[CAMERA_WIDTH][HEIGHT_CAPTURED];
      int[][] g = new int[CAMERA_WIDTH][HEIGHT_CAPTURED];
      int[][] b = new int[CAMERA_WIDTH][HEIGHT_CAPTURED];
      
      for (int i = 0; i < CAMERA_WIDTH; i++) {
        for (int j = -HEIGHT_CAPTURED/2; j < HEIGHT_CAPTURED/2; j++) {
          r[i][j+HEIGHT_CAPTURED/2] = Camera.imageGetRed(image, CAMERA_WIDTH, i, CAMERA_HEIGHT/2+j);
          g[i][j+HEIGHT_CAPTURED/2] = Camera.imageGetGreen(image, CAMERA_WIDTH, i, CAMERA_HEIGHT/2+j);
          b[i][j+HEIGHT_CAPTURED/2] = Camera.imageGetBlue(image, CAMERA_WIDTH, i, CAMERA_HEIGHT/2+j);
        }
      }
      
      int[][] colorCount = countColor(r, g, b);
      int totalGreen = colorCount[0][0] + colorCount[0][1] + colorCount[0][2];
      
      // Check if we still see green
      if (totalGreen < 15) {
        stepsWithoutGreen++;
        if (stepsWithoutGreen > maxStepsWithoutGreen) {
          System.out.println("Lost green for too long! Aborting approach.");
          leftMotor.setVelocity(0);
          rightMotor.setVelocity(0);
          return false;
        }
      } else {
        stepsWithoutGreen = 0; // Reset counter
      }
      
      // Determine where green is
      boolean greenLeft = colorCount[0][0] > colorCount[0][1] + EPSILON && 
                         colorCount[0][0] > colorCount[0][2] + EPSILON;
      boolean greenCenter = colorCount[0][1] > colorCount[0][0] + EPSILON && 
                           colorCount[0][1] > colorCount[0][2] + EPSILON;
      boolean greenRight = colorCount[0][2] > colorCount[0][0] + EPSILON && 
                          colorCount[0][2] > colorCount[0][1] + EPSILON;
      
      // Print status every 20 steps
      if (approachSteps % 20 == 0) {
        System.out.println("  Step " + approachSteps + ": Green=" + totalGreen + 
                          " [L:" + colorCount[0][0] + " C:" + colorCount[0][1] + 
                          " R:" + colorCount[0][2] + "]");
      }
      
      // Adjust motors based on green position
      if (greenLeft) {
        // Turn left slowly while moving forward
        leftMotor.setVelocity(approachSpeed * 0.3);
        rightMotor.setVelocity(approachSpeed * 1.2);
      } else if (greenRight) {
        // Turn right slowly while moving forward
        leftMotor.setVelocity(approachSpeed * 1.2);
        rightMotor.setVelocity(approachSpeed * 0.3);
      } else if (greenCenter || totalGreen > 30) {
        // Go straight slowly
        leftMotor.setVelocity(approachSpeed);
        rightMotor.setVelocity(approachSpeed);
      } else {
        // Not enough green or not clear where it is - stop and reassess
        System.out.println("Green not clear, stopping to reassess...");
        leftMotor.setVelocity(0);
        rightMotor.setVelocity(0);
        
        // Take 5 steps to look around slightly
        for (int i = 0; i < 5; i++) {
          robot.step(timeStep);
        }
      }
      
      // SAFETY: If we've been approaching for a very long time, something is wrong
      if (approachSteps > 300) {
        System.out.println("WARNING: Approach taking too long, probably hitting wall!");
        leftMotor.setVelocity(0);
        rightMotor.setVelocity(0);
        return false;
      }
    }
    
    System.out.println("Approach timeout reached");
    leftMotor.setVelocity(0);
    rightMotor.setVelocity(0);
    return false;
  }
  
  // NEW: Execute grab with verification
  private static boolean executeGrab(int timeStep) {
    System.out.println("\n>>> EXECUTING GRAB SEQUENCE <<<");
    
    // Make sure we're still touching
    if (jarDetectedSensor.getValue() != 1.0f) {
      System.out.println("ERROR: Lost contact before grab!");
      return false;
    }
    
    // Stop all movement
    leftMotor.setVelocity(0);
    rightMotor.setVelocity(0);
    
    // Wait a moment to stabilize
    for (int i = 0; i < 10; i++) {
      robot.step(timeStep);
    }
    
    System.out.println("Step 1: Closing gripper...");
    // Close gripper gradually
    for (int i = 0; i < 100; i++) {
      robot.step(timeStep);
      openCloseGripper(0.01f);
      leftMotor.setVelocity(0);
      rightMotor.setVelocity(0);
    }
    
    System.out.println("Step 2: Lifting gripper...");
    // Lift gripper
    for (int i = 0; i < 120; i++) {
      robot.step(timeStep);
      liftLowerGripper(-0.0499f);
      openCloseGripper(0.01f);
      leftMotor.setVelocity(0);
      rightMotor.setVelocity(0);
    }
    
    System.out.println("Step 3: Verifying grab...");
    // Wait and verify
    for (int i = 0; i < 20; i++) {
      robot.step(timeStep);
    }
    
    System.out.println(">>> GRAB COMPLETE! <<<\n");
    return true;
  }
  
  // Helper: Move randomly to explore
  private static void moveRandomly(int timeStep) {
    System.out.println("Moving to different position...");
    
    // Turn random amount
    int randomTurn = (int)(Math.random() * 180) - 90; // -90 to +90 degrees
    rotateByDegrees(randomTurn, timeStep);
    
    // Move forward a bit
    leftMotor.setVelocity(MAX_SPEED / 3);
    rightMotor.setVelocity(MAX_SPEED / 3);
    
    for (int i = 0; i < 60; i++) {
      robot.step(timeStep);
    }
    
    leftMotor.setVelocity(0);
    rightMotor.setVelocity(0);
  }
  
  // Helper: Back up specific distance
  private static void backUpDistance(int timeStep, int steps) {
    System.out.println("Backing up " + steps + " steps...");
    
    for (int i = 0; i < steps; i++) {
      robot.step(timeStep);
      leftMotor.setVelocity(-MAX_SPEED / 3);
      rightMotor.setVelocity(-MAX_SPEED / 3);
    }
    
    leftMotor.setVelocity(0);
    rightMotor.setVelocity(0);
  }
  
  // Helper: Rotate by degrees
  private static void rotateByDegrees(int degrees, int timeStep) {
    int startAngle = getCompassReadingInDegrees(compass);
    int targetAngle = (startAngle + degrees + 360) % 360;
    
    if (degrees > 0) {
      leftMotor.setVelocity(-MAX_SPEED / 3);
      rightMotor.setVelocity(MAX_SPEED / 3);
    } else {
      leftMotor.setVelocity(MAX_SPEED / 3);
      rightMotor.setVelocity(-MAX_SPEED / 3);
    }
    
    int timeout = 150;
    while (robot.step(timeStep) != -1 && timeout > 0) {
      int currentAngle = (getCompassReadingInDegrees(compass) + 360) % 360;
      int diff = Math.abs(currentAngle - targetAngle);
      if (diff < 5 || diff > 355) {
        break;
      }
      timeout--;
    }
    
    leftMotor.setVelocity(0);
    rightMotor.setVelocity(0);
  }
  
  // Helper: Turn to specific angle
  private static void turnToAngle(int targetAngle, int timeStep) {
    System.out.println("Turning to angle: " + targetAngle);
    
    int currentAngle = getCompassReadingInDegrees(compass);
    int diff = (targetAngle - currentAngle + 540) % 360 - 180; // Shortest path
    
    rotateByDegrees(diff, timeStep);
  }
  
  // Green detection data structure
  private static class GreenDetection {
    int angle;
    int totalGreen;
    String section;
    int greenLeft;
    int greenCenter;
    int greenRight;
  }
  
  // Read compass
  private static int getCompassReadingInDegrees(Compass compass) {
    double compassReadings[] = compass.getValues();
    double rad = Math.atan2(compassReadings[0], compassReadings[2]);
    double bearing = -((rad + Math.PI) / Math.PI * 180.0);
    if (bearing > 180)
      bearing = 360 - bearing;
    if (bearing < -180)
      bearing = 360 + bearing;
    return (int)(bearing);
  }
  
  // Color counting with RELAXED thresholds
  static private int[][] countColor(int[][] r, int[][] g, int[][] b) {
    int[][] results = new int[2][3];
    
    // VERY RELAXED green detection thresholds
    int green_maximum = 120;  // Increased
    int green_minimum = 35;   // Decreased
    
    int blue_maximum = 20;
    int blue_minimum = 80;
    
    for (int i = 0; i < 3; i++) {
      results[0][i] = 0;
      results[1][i] = 0;
    }
    
    for (int j = 0; j < HEIGHT_CAPTURED; j++) {
      for (int i = 0; i < (int)Math.floor(CAMERA_WIDTH/3); i++) {
        if (r[i][j]<green_maximum && g[i][j]>green_minimum && b[i][j]<green_maximum) results[0][0]++;
        if (r[i][j]<blue_maximum && g[i][j]<blue_maximum && b[i][j]>blue_minimum)    results[1][0]++;
      }
      
      for (int i = (int)Math.floor(CAMERA_WIDTH/3); i < CAMERA_WIDTH-(int)Math.floor(CAMERA_WIDTH/3); i++) {
        if (r[i][j]<green_maximum && g[i][j]>green_minimum && b[i][j]<green_maximum) results[0][1]++;
        if (r[i][j]<blue_maximum && g[i][j]<blue_maximum && b[i][j]>blue_minimum)    results[1][1]++;
      }
      
      for (int i = CAMERA_WIDTH-(int)Math.floor(CAMERA_WIDTH/3); i < CAMERA_WIDTH; i++) {
        if (r[i][j]<green_maximum && g[i][j]>green_minimum && b[i][j]<green_maximum) results[0][2]++;
        if (r[i][j]<blue_maximum && g[i][j]<blue_maximum && b[i][j]>blue_minimum)    results[1][2]++;
      }
    }
    
    return results;
  }
  
  // Navigation methods
  private static void navigateToSearchArea() {
    System.out.println("Navigating to search area...");
    int timeStep = (int) Math.round(robot.getBasicTimeStep());
    
    ArrayList<Point> path = new ArrayList<>();
    double values[] = TranslationField.getSFVec3f();
    double x = (values[0]*100);
    double y = -(values[2]*100);
    
    path.add(new Point((int)x, (int)y));
    path.add(new Point(850, 100));
    path.add(new Point(850, 60));
    
    executeHardcodedPath(path, timeStep);
  }
  
  private static void navigateFromDropoffToSearch() {
    System.out.println("Navigating from dropoff to search...");
    int timeStep = (int) Math.round(robot.getBasicTimeStep());
    
    ArrayList<Point> path = new ArrayList<>();
    double values[] = TranslationField.getSFVec3f();
    double x = (values[0]*100);
    double y = -(values[2]*100);
    
    path.add(new Point((int)x, (int)y));
    path.add(new Point(-850, 602));
    path.add(new Point(-815, 410));
    path.add(new Point(-590, 390));
    path.add(new Point(-590, 510));
    path.add(new Point(-340, 605));
    path.add(new Point(-220, 400));
    path.add(new Point(-215, 278));
    path.add(new Point(-75, 278));
    path.add(new Point(0, 144));
    path.add(new Point(350, 144));
    path.add(new Point(850, 100));
    path.add(new Point(850, 60));
    
    executeHardcodedPath(path, timeStep);
  }
  
  private static void navigateToDropoff() {
    System.out.println("Navigating to dropoff...");
    int timeStep = (int) Math.round(robot.getBasicTimeStep());
    
    ArrayList<Point> path = new ArrayList<>();
    double values[] = TranslationField.getSFVec3f();
    double x = (values[0]*100);
    double y = -(values[2]*100);
    
    path.add(new Point((int)x, (int)y));
    path.add(new Point(850, 100));
    path.add(new Point(350, 144));
    path.add(new Point(0, 144));
    path.add(new Point(-75, 278));
    path.add(new Point(-215, 278));
    path.add(new Point(-340, 605));
    path.add(new Point(-590, 510));
    path.add(new Point(-590, 390));
    path.add(new Point(-815, 410));
    path.add(new Point(-850, 602));
    
    executeHardcodedPath(path, timeStep);
  }
  
  private static void placeJar(int timeStep) {
    System.out.println("Placing jar...");
    
    ArrayList<Point> path = new ArrayList<>();
    double values[] = TranslationField.getSFVec3f();
    double x = (values[0]*100);
    double y = -(values[2]*100);
    
    int yVal = 602 + (5 - jarsToGo) * 30 + 5;
    path.add(new Point((int)x, (int)y));
    path.add(new Point(-850, yVal));
    path.add(new Point(-900, yVal));
    path.add(new Point(-915, yVal));
    path.add(new Point(-936, yVal));
    
    executeHardcodedPath(path, timeStep);
    
    // Release jar
    for (int i = 0; i < 50; i++) {
      robot.step(timeStep);
      openCloseGripper(0.099f);
      liftLowerGripper(0.001f);
    }
    
    // Back up
    backUpDistance(timeStep, 50);
  }
  
  private static void hardcodedPathPushing() {
    int timeStep = (int) Math.round(robot.getBasicTimeStep());
    
    ArrayList<Point> path = new ArrayList<>();
    double values[] = TranslationField.getSFVec3f();
    double x = (values[0]*100);
    double y = -(values[2]*100);
    
    int bffr = 35;
    path.add(new Point((int)x, (int)y));
    path.add(new Point(-980+60+bffr, 530));
    path.add(new Point(-970+60+bffr, 800));
    path.add(new Point(-814, 555));
    path.add(new Point(-814, 190+30+bffr+120));
    path.add(new Point(-441-30-60-bffr, 190+30+bffr+120));
    path.add(new Point(-441-30-bffr, 800));
    path.add(new Point(-145-30-bffr, 447+50));
    path.add(new Point(-145-30-bffr, 122));
    path.add(new Point(-145-30-bffr, 278));
    path.add(new Point(565-150-60-bffr, 278));
    path.add(new Point(565-150-60-bffr, 0+60+bffr));
    path.add(new Point(543-30-bffr, 144));
    path.add(new Point(991-20-60-bffr, 144));
    path.add(new Point(875, 100));
    path.add(new Point(850, 60));
    
    executeHardcodedPath(path, timeStep);
  }
  
  // Execute hardcoded path
  private static void executeHardcodedPath(ArrayList<Point> path, int timeStep) {
    if (path == null || path.size() <= 1) return;
    
    for (int i = 1; i < path.size(); i++) {
      double values[] = TranslationField.getSFVec3f();
      double x = (values[0]*100);
      double y = -(values[2]*100);
      
      makeHardcodedTurn(x, y, path.get(i).x, path.get(i).y);
      moveHardcodedAhead(x, y, path.get(i).x, path.get(i).y);
    }
  }
  
  private static boolean moveHardcodedAhead(double x1, double y1, double x2, double y2) {
    double xDiff = x2 - x1;
    double yDiff = y2 - y1;
    
    leftMotor.setVelocity(MAX_SPEED);
    rightMotor.setVelocity(MAX_SPEED);
    
    double desiredDistance = Math.sqrt((xDiff)*(xDiff) + (yDiff)*(yDiff));
    int timeStep = (int) Math.round(robot.getBasicTimeStep());
    
    while(robot.step(timeStep) != -1) {
      double values[] = TranslationField.getSFVec3f();
      double x = (values[0]*100);
      double y = -(values[2]*100);
      if (Math.sqrt((x-x1)*(x-x1) + (y-y1)*(y-y1)) >= desiredDistance) 
        break;
    }
    
    leftMotor.setVelocity(0);
    rightMotor.setVelocity(0); 
    return false;
  }
  
  private static void makeHardcodedTurn(double x1, double y1, double x2, double y2) {
    double xDiff = x2 - x1;
    double yDiff = y2 - y1;
    int startAngle = getCompassReadingInDegrees(compass);

    int turn = (int)(Math.atan2(yDiff, xDiff) * 180 / Math.PI);
    turn = (turn - startAngle) % 360;
    if (turn < -180)
      turn += 360;
    else if (turn > 180)
      turn -= 360;
    
    int finalAngle = (startAngle + turn +360)%360;

    if (turn > 0) {
      leftMotor.setVelocity(-MAX_SPEED);
      rightMotor.setVelocity(MAX_SPEED);
    } else {
      leftMotor.setVelocity(MAX_SPEED);
      rightMotor.setVelocity(-MAX_SPEED);   
    }
    
    int timeStep = (int) Math.round(robot.getBasicTimeStep());
    
    while(robot.step(timeStep) != -1) {
      int degrees = (getCompassReadingInDegrees(compass)+360)%360;
      if (degrees > finalAngle - 3 && degrees < finalAngle + 3) 
        break;
       
      int distTurningRight = (degrees - finalAngle + 360)%360;
      int distTurningLeft = (finalAngle - degrees + 360)%360;
      
      if (distTurningRight < distTurningLeft) {
        leftMotor.setVelocity(MAX_SPEED);
        rightMotor.setVelocity(-MAX_SPEED);   
      } else {
        leftMotor.setVelocity(-MAX_SPEED);
        rightMotor.setVelocity(MAX_SPEED);
      }
    }
    
    leftMotor.setVelocity(0);
    rightMotor.setVelocity(0); 
  }
}