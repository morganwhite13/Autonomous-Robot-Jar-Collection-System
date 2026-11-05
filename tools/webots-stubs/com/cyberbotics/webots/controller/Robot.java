package com.cyberbotics.webots.controller;

// Compile-only stub of the Webots R2025a Java API (signatures from javap on Controller.jar).
// Lets CI type-check and analyse the controllers without a Webots install. Never run:
// methods return neutral values so static analysis sees the same flow as with the real API.
public class Robot {
  public Robot() {}
  public int step(int duration) { return 0; }
  public double getBasicTimeStep() { return 0; }
  public double getTime() { return 0; }
  public Motor getMotor(String name) { return new Motor(); }
  public DistanceSensor getDistanceSensor(String name) { return new DistanceSensor(); }
  public Compass getCompass(String name) { return new Compass(); }
  public Camera getCamera(String name) { return new Camera(name); }
  public TouchSensor getTouchSensor(String name) { return new TouchSensor(name); }
}
