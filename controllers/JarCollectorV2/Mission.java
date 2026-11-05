import java.util.List;
import java.util.Random;

/**
 * The mission as an explicit sequence: start-up, clear boxes, then for each jar
 * travel to search point, search and grab, carry to drop-off, place.
 *
 * <p>Fix vs v1: v1 picked the route to the search point with {@code if (jarsToGo == 5)}, so after
 * a failed search for jars 2-5 it used the drop-off route even though the robot was still at the
 * search point. That route's first leg is a straight line from the jar bay to the drop-off,
 * diagonally across the arena through the static wooden boxes. v2 tracks where the robot is.
 * v1 also retried failed searches forever; v2 stops after
 * {@link Config#MAX_CONSECUTIVE_FAILED_SEARCHES} and reports what it achieved.
 */
final class Mission {

  static final String PHASE_STARTUP = "Start-up";
  static final String PHASE_CLEARING = "Clearing path";
  static final String PHASE_TO_SEARCH = "Travel to search point";
  static final String PHASE_SEARCH = "Search and grab";
  static final String PHASE_TO_DROPOFF = "Carry to drop-off";
  static final String PHASE_PLACE = "Place jar";

  private final RobotPlatform robot;
  private final MissionStats stats;
  private final Navigator nav;
  private final JarHandler jars;

  private final Routes.RouteSet routes;

  Mission(RobotPlatform robot) {
    this(robot, Config.USE_PLANNED_ROUTES ? Routes.PLANNED : Routes.HAND_TUNED);
  }

  Mission(RobotPlatform robot, Routes.RouteSet routes) {
    this.robot = robot;
    this.routes = routes;
    this.stats = new MissionStats(robot::elapsedMs);
    this.nav = new Navigator(robot, stats);
    this.jars = new JarHandler(robot, nav, stats, new Random(Config.RANDOM_SEED));
  }

  MissionStats stats() {
    return stats;
  }

  /** Runs the whole mission and returns the number of jars delivered. */
  int run() {
    startUp();
    if (routes != Routes.HAND_TUNED) {
      Log.info("Using the " + routes.name + " carry and return routes");
    }

    Log.phase("PHASE 1: Clearing path");
    stats.beginPhase(PHASE_CLEARING);
    nav.followRoute(Routes.CLEARING_PATH);

    boolean atDropoff = false;
    int delivered = 0;
    int failedInARow = 0;
    while (delivered < Config.JAR_COUNT) {
      int jarNumber = delivered + 1;
      Log.phase("COLLECTING JAR " + jarNumber + " of " + Config.JAR_COUNT);
      stats.beginJar(jarNumber);

      stats.beginPhase(PHASE_TO_SEARCH);
      List<Vec2> route = atDropoff ? routes.dropoffToSearch : Routes.TO_SEARCH_POINT;
      nav.followRoute(route);
      atDropoff = false;

      stats.beginPhase(PHASE_SEARCH);
      if (!jars.searchAndGrab()) {
        stats.recordFailedSearch();
        failedInARow++;
        Log.warn("search failed (" + failedInARow + " in a row)");
        if (failedInARow >= Config.MAX_CONSECUTIVE_FAILED_SEARCHES) {
          Log.warn("giving up after " + failedInARow + " failed searches");
          break;
        }
        continue; // still near the search point, so the short route is used next time
      }
      failedInARow = 0;

      stats.beginPhase(PHASE_TO_DROPOFF);
      nav.followRoute(routes.searchToDropoff);

      stats.beginPhase(PHASE_PLACE);
      jars.place(delivered);
      atDropoff = true;
      delivered++;
      stats.jarDelivered();
      Log.info("Jars remaining: " + (Config.JAR_COUNT - delivered));
    }

    stats.endPhase();
    Log.phase(delivered == Config.JAR_COUNT ? "ALL JARS COLLECTED!" : "MISSION INCOMPLETE");
    Log.block(stats.report());
    return delivered;
  }

  private void startUp() {
    stats.beginPhase(PHASE_STARTUP);
    Log.phase("Start-up: opening and lowering gripper");
    robot.setFingers(Config.FINGERS_OPEN);
    robot.setLift(Config.LIFT_DOWN);
    int settleSteps = (int) Math.ceil((double) Config.STARTUP_SETTLE_MS / robot.timeStepMs());
    robot.waitSteps(Math.max(1, settleSteps));
    if (Config.PRINT_SENSOR_SELF_TEST) {
      robot.waitSteps(Config.SENSOR_SELF_TEST_STEPS);
      Log.info("Sensor self-test: " + robot.diagnostics());
    }
  }
}
