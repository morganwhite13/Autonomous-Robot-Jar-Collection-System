import com.cyberbotics.webots.controller.*;
import java.lang.reflect.Method;

/** Runs makeHardcodedTurn from every start heading to every target heading (10-degree grid)
 *  and checks it stops within 3 degrees of the target, measured around the circle. */
public class TurnTest {
  static int reading(Method m, Compass c) throws Exception { return ((int) m.invoke(null, c) + 360) % 360; }
  public static void main(String[] a) throws Exception {
    double perStep = Double.parseDouble(a[0]);  // radians per step per unit speed difference
    Class<?> k = Class.forName("ProjectController3");
    java.lang.reflect.Field robot = k.getDeclaredField("robot"); robot.setAccessible(true); robot.set(null, new Supervisor());
    for (String f : new String[] {"leftMotor", "rightMotor"}) {
      java.lang.reflect.Field fm = k.getDeclaredField(f); fm.setAccessible(true);
      fm.set(null, new Motor(f.equals("leftMotor") ? "left wheel" : "right wheel"));
    }
    Compass c = new Compass();
    java.lang.reflect.Field fc = k.getDeclaredField("compass"); fc.setAccessible(true); fc.set(null, c);
    Method turn = k.getDeclaredMethod("makeHardcodedTurn", double.class, double.class, double.class, double.class); turn.setAccessible(true);
    Method read = k.getDeclaredMethod("getCompassReadingInDegrees", Compass.class); read.setAccessible(true);
    Sim.radPerStepPerSpeed = perStep;
    int bad = 0, worstSteps = 0, runs = 0;
    for (int start = 0; start < 360; start += 10) for (int target = 0; target < 360; target += 10) {
      // set the compass so the controller reads `start`
      double best = 0; int bestErr = 999;
      for (int i = 0; i < 3600; i++) { Sim.rad = i * Math.PI / 1800; int e = Math.abs(reading(read, c) - start); e = Math.min(e, 360 - e); if (e < bestErr) { bestErr = e; best = Sim.rad; } }
      Sim.rad = best; Sim.steps = 0; Sim.maxSteps = 20000;
      double t = Math.toRadians(target);
      turn.invoke(null, 0.0, 0.0, 100 * Math.cos(t), 100 * Math.sin(t));
      int got = reading(read, c), err = Math.abs(got - target); err = Math.min(err, 360 - err);
      runs++; worstSteps = Math.max(worstSteps, Sim.steps);
      if (Sim.steps >= Sim.maxSteps || err > 4) { bad++; if (bad <= 5) System.out.println("FAIL start=" + start + " target=" + target + " got=" + got + " steps=" + Sim.steps); }
    }
    System.out.println("turn rate " + perStep + ": " + (runs - bad) + "/" + runs + " turns ended within 4 degrees; worst case " + worstSteps + " steps");
  }
}
