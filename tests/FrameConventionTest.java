import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.nio.file.Path;
import org.junit.jupiter.api.Test;

/**
 * Checks v2's heading and position conventions against Webots' own definitions, independently of
 * v1. (AnglesTest only shows v2 matches v1's formula; a mistake shared with v1 would pass that.)
 *
 * <p>Facts used, from the world file and the Pioneer 3-DX proto: the world is NUE, so north is
 * world +X and up is +Y; the robot's forward axis is its local -Z (bumper, camera and gripper sit
 * at negative z); the compass is mounted without rotation, so it reports the north direction in
 * the robot's frame. For a robot turned by yaw psi about +Y, that is R(psi)^T * (1, 0, 0)
 * = (cos psi, 0, sin psi).
 */
class FrameConventionTest {

  /** Compass reading for a robot rotated by {@code yawDeg} about the world's up axis. */
  private static double[] compassAt(double yawDeg) {
    double psi = Math.toRadians(yawDeg);
    return new double[] {Math.cos(psi), 0, Math.sin(psi)};
  }

  /** Where the robot's forward axis (local -Z) points in the world, for the same yaw. */
  private static double[] forwardAt(double yawDeg) {
    double psi = Math.toRadians(yawDeg);
    // R_y(psi) * (0, 0, -1) = (-sin psi, 0, -cos psi)
    return new double[] {-Math.sin(psi), 0, -Math.cos(psi)};
  }

  @Test
  void compassHeadingIsTheDirectionTheRobotDrives() {
    for (int yaw = -180; yaw < 540; yaw += 7) {
      Vec2 origin = Vec2.fromWorld(new double[] {0, 0, 0});
      Vec2 ahead = Vec2.fromWorld(forwardAt(yaw));
      double drivingDirection = origin.bearingTo(ahead);
      assertTrue(Angles.within(Angles.fromCompass(compassAt(yaw)), drivingDirection, 1e-9),
          "yaw " + yaw);
    }
  }

  @Test
  void robotInTheWorldFileStartsFacingPlusY() {
    // The Pioneer3dx node's rotation angle is 0 (identity), so it faces local -Z = +y here.
    assertEquals(90, Angles.fromCompass(compassAt(0)), 1e-9);
  }

  @Test
  void turningLeftIncreasesTheHeading() {
    // A left (counter-clockwise from above) turn is a positive rotation about +Y.
    double before = Angles.fromCompass(compassAt(0));
    double after = Angles.fromCompass(compassAt(10));
    assertEquals(10, Angles.delta(before, after), 1e-9);
  }

  @Test
  void positionMappingMatchesTheWorldFile() throws Exception {
    WorldFile world = WorldFile.read(Path.of("worlds/ProjectWorld2025.wbt"));
    // Pioneer3dx translation -8.1565 0.0945536 -5.56873
    Vec2 fromTranslation = Vec2.fromWorld(new double[] {-8.1565, 0.0945536, -5.56873});
    assertTrue(world.robotStart.distanceTo(fromTranslation) < 1e-9);
    assertEquals(-815.65, world.robotStart.x, 1e-9);
    assertEquals(556.873, world.robotStart.y, 1e-9);
  }
}
