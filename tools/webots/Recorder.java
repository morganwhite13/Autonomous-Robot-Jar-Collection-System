import com.cyberbotics.webots.controller.Camera;
import com.cyberbotics.webots.controller.Robot;

/**
 * Saves a frame from a camera looking straight down on the arena every few simulated seconds,
 * for tools/make-demo-gif.sh. Added only to the copies of the worlds that
 * tools/run-in-webots.sh --frames makes.
 * controllerArgs: [output directory, seconds between frames].
 */
public class Recorder {
  public static void main(String[] args) {
    String out = args[0];
    double period = Double.parseDouble(args[1]);
    Robot robot = new Robot();
    int timeStep = (int) Math.round(robot.getBasicTimeStep());
    // Render only when a frame is due: rendering 640x400 every step slowed runs about tenfold.
    int periodMs = Math.max(1, (int) Math.round(period * 1000 / timeStep)) * timeStep;
    Camera camera = robot.getCamera("arena camera");
    camera.enable(periodMs);
    int frame = 0;
    while (robot.step(periodMs) != -1) {
      camera.saveImage(String.format("%s/frame%05d.png", out, frame++), 90);
    }
  }
}
