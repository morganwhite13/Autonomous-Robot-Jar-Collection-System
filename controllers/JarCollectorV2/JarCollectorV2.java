/**
 * Webots entry point for the v2 controller (worlds/ProjectWorld2025_v2.wbt).
 *
 * <p>The original controller is kept unchanged in controllers/ProjectController3.
 * See docs/BEFORE_AND_AFTER.md for what changed and why.
 */
public class JarCollectorV2 {

  public static void main(String[] args) {
    WebotsPlatform robot = new WebotsPlatform();
    Log.useClock(robot::elapsedMs);
    try {
      new Mission(robot).run();
    } catch (RobotPlatform.SimulationEnded e) {
      Log.info("Simulation ended by Webots; controller exiting.");
    }
  }
}
