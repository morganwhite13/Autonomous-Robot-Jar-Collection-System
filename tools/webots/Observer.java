import com.cyberbotics.webots.controller.Field;
import com.cyberbotics.webots.controller.Node;
import com.cyberbotics.webots.controller.Supervisor;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Supervisor that watches a run without touching it: it logs the robot's path, tracks when each
 * jar reaches the drop-off ({@link DeliveryTracker}), and ends the simulation when all jars are
 * delivered or the time limit is reached.
 *
 * <p>It exists only in the temporary worlds that tools/run-in-webots.sh builds (which add
 * {@code DEF ROBOT} / {@code DEF JAR0..4} and this node); the real worlds don't contain it.
 * controllerArgs: [output directory, simulated-time limit in seconds].
 */
public class Observer {

  public static void main(String[] args) throws IOException {
    Supervisor supervisor = new Supervisor();
    int timeStep = (int) Math.round(supervisor.getBasicTimeStep());
    Path outDir = Paths.get(args.length > 0 ? args[0] : ".");
    double limitS = args.length > 1 ? Double.parseDouble(args[1]) : 3600;

    Node robotNode = supervisor.getFromDef("ROBOT");
    Field robot = robotNode.getField("translation");
    List<Node> jarNodes = new ArrayList<>();
    List<Field> jars = new ArrayList<>();
    for (Node jar = supervisor.getFromDef("JAR0"); jar != null;
        jar = supervisor.getFromDef("JAR" + jars.size())) {
      jarNodes.add(jar);
      jars.add(jar.getField("translation"));
    }
    int[] upAxis = new int[jarNodes.size()];
    for (int i = 0; i < jarNodes.size(); i++) {
      upAxis[i] = upAxis(jarNodes.get(i).getOrientation());
    }
    DeliveryTracker tracker = new DeliveryTracker(jars.size());
    System.out.println("[observer] watching " + jars.size() + " jars, limit " + limitS + " s");

    double time = 0;
    boolean ended = false;
    try (PrintWriter path = new PrintWriter(
        Files.newBufferedWriter(outDir.resolve("trajectory.csv"), StandardCharsets.UTF_8));
        PrintWriter poses = new PrintWriter(
            Files.newBufferedWriter(outDir.resolve("poses.csv"), StandardCharsets.UTF_8))) {
      path.println("t,x,y");
      poses.println("t,x,y,heading"); // heading in degrees, as the controllers' compass gives it
      double nextSample = 0;
      int lastReported = 0;
      while (true) {
        if (supervisor.step(timeStep) == -1) {
          ended = true;
          break;
        }
        time = supervisor.getTime();
        double[] r = robot.getSFVec3f();
        if (time >= nextSample) {
          path.printf(Locale.ROOT, "%.2f,%.1f,%.1f%n", time, r[0] * 100, -r[2] * 100);
          // The robot drives along its local -Z: in the world that is minus the third column.
          double[] m = robotNode.getOrientation();
          double heading = Math.toDegrees(Math.atan2(m[8], -m[2])); // controllers' y is -Z
          poses.printf(Locale.ROOT, "%.2f,%.1f,%.1f,%.1f%n", time, r[0] * 100, -r[2] * 100,
              heading);
          nextSample += 0.5;
        }
        for (int i = 0; i < jars.size(); i++) {
          double[] p = jars.get(i).getSFVec3f();
          tracker.update(i, time, p[0] * 100, -p[2] * 100);
        }
        if (tracker.delivered() != lastReported) {
          lastReported = tracker.delivered();
          System.out.printf(Locale.ROOT, "[observer] %.1f s: %d/%d jars delivered%n",
              time, lastReported, jars.size());
        }
        boolean done = tracker.allDelivered() && time - tracker.lastDeliveryTime() > 2;
        if (done || time >= limitS) {
          break;
        }
      }
    }
    Files.write(outDir.resolve("result.properties"),
        tracker.toProperties(time, time >= limitS).getBytes(StandardCharsets.UTF_8));
    writeFinalJarPositions(outDir.resolve("jars.csv"), jarNodes, upAxis);
    System.out.printf(Locale.ROOT, "[observer] finished at %.1f s with %d/%d jars delivered%n",
        time, tracker.delivered(), jars.size());
    if (!ended) {
      supervisor.simulationQuit(0);
    }
  }

  /**
   * Where each jar is when the run ends, and whether it is still standing: tells a jar the robot
   * never reached from one it knocked over or left somewhere awkward.
   */
  private static void writeFinalJarPositions(Path file, List<Node> jars, int[] upAxis)
      throws IOException {
    List<String> lines = new ArrayList<>();
    lines.add("jar,x,y,upright");
    for (int i = 0; i < jars.size(); i++) {
      double[] p = jars.get(i).getPosition();
      double[] m = jars.get(i).getOrientation();
      // The axis that pointed up at the start still points within 25 degrees of straight up.
      int axis = Math.abs(upAxis[i]) - 1;
      boolean upright = Math.signum(upAxis[i]) * m[3 + axis] > 0.9;
      lines.add(String.format(Locale.ROOT, "%d,%.1f,%.1f,%b", i, p[0] * 100, -p[2] * 100,
          upright));
      if (!upright) {
        System.out.printf(Locale.ROOT, "[observer] jar %d is lying at (%.0f, %.0f)%n", i,
            p[0] * 100, -p[2] * 100);
      }
    }
    Files.write(file, lines, StandardCharsets.UTF_8);
  }

  /**
   * Which of a node's own axes points up (world +Y), from its orientation matrix: 1, 2 or 3 for
   * X, Y or Z, negative if that axis points down. The jars are rotated in the world, so their up
   * axis isn't their Y.
   */
  static int upAxis(double[] m) {
    int best = 0;
    for (int k = 1; k < 3; k++) {
      if (Math.abs(m[3 + k]) > Math.abs(m[3 + best])) {
        best = k;
      }
    }
    return (int) Math.signum(m[3 + best]) * (best + 1);
  }
}
