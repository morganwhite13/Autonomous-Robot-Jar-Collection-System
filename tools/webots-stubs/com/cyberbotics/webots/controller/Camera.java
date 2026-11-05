package com.cyberbotics.webots.controller;

// Compile-only stub of the Webots R2025a Java API (signatures from javap on Controller.jar).
// Lets CI type-check and analyse the controllers without a Webots install. Never run:
// methods return neutral values so static analysis sees the same flow as with the real API.
public class Camera extends Device {
  public Camera(String name) {}
  public void enable(int samplingPeriod) { }
  public int[] getImage() { return new int[0]; }
  public int getWidth() { return 0; }
  public int getHeight() { return 0; }
  public static int imageGetRed(int[] image, int width, int x, int y) { return 0; }
  public static int imageGetGreen(int[] image, int width, int x, int y) { return 0; }
  public static int imageGetBlue(int[] image, int width, int x, int y) { return 0; }
}
