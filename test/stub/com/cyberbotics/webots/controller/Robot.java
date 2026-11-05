package com.cyberbotics.webots.controller;
public class Robot {
  public int step(int t) { Sim.steps++; if (Sim.steps > Sim.maxSteps) return -1;
    Sim.rad -= (Sim.vr - Sim.vl) * Sim.radPerStepPerSpeed; return 0; }
  public double getBasicTimeStep() { return 32; }
  public Motor getMotor(String n) { return new Motor(n); }
  public DistanceSensor getDistanceSensor(String n) { return new DistanceSensor(); }
  public Compass getCompass(String n) { return new Compass(); }
}
