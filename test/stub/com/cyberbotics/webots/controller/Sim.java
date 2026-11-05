package com.cyberbotics.webots.controller;
/** Tiny kinematic stand-in for Webots: heading changes with the wheel speed difference. */
public class Sim {
  public static double rad = 0;           // compass angle state
  public static double vl = 0, vr = 0;    // wheel velocities
  public static double radPerStepPerSpeed = 0.002;
  public static int steps = 0, maxSteps = 100000;
}
