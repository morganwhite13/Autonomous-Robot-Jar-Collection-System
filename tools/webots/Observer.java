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

    Field robot = supervisor.getFromDef("ROBOT").getField("translation");
    List<Field> jars = new ArrayList<>();
    for (Node jar = supervisor.getFromDef("JAR0"); jar != null;
        jar = supervisor.getFromDef("JAR" + jars.size())) {
      jars.add(jar.getField("translation"));
    }
    DeliveryTracker tracker = new DeliveryTracker(jars.size());
    System.out.println("[observer] watching " + jars.size() + " jars, limit " + limitS + " s");

    double time = 0;
    boolean ended = false;
    try (PrintWriter path = new PrintWriter(
        Files.newBufferedWriter(outDir.resolve("trajectory.csv"), StandardCharsets.UTF_8))) {
      path.println("t,x,y");
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
    System.out.printf(Locale.ROOT, "[observer] finished at %.1f s with %d/%d jars delivered%n",
        time, tracker.delivered(), jars.size());
    if (!ended) {
      supervisor.simulationQuit(0);
    }
  }
}
