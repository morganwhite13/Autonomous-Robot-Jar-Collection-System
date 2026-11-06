package com.cyberbotics.webots.controller;

// Compile-only stub of the Webots R2025a Java API (signatures from javap on Controller.jar).
// Lets CI type-check and analyse the controllers without a Webots install. Never run:
// methods return neutral values so static analysis sees the same flow as with the real API.
public class Node {
  public Field getField(String fieldName) { return new Field(); }
  public double[] getPosition() { return new double[3]; }
  public double[] getOrientation() { return new double[] {1, 0, 0, 0, 1, 0, 0, 0, 1}; }
}
