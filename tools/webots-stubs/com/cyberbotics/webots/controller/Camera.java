package com.cyberbotics.webots.controller;

// Compile-only stub of the Webots R2025a Java API (signatures from javap on Controller.jar).
// Lets CI type-check WebotsPlatform without a Webots install. Never used at runtime.
public class Camera extends Device {
  public Camera(String name) {}
  public void enable(int samplingPeriod) { throw new UnsupportedOperationException("Webots API stub"); }
  public int[] getImage() { throw new UnsupportedOperationException("Webots API stub"); }
  public int getWidth() { throw new UnsupportedOperationException("Webots API stub"); }
  public int getHeight() { throw new UnsupportedOperationException("Webots API stub"); }
  public static int imageGetRed(int[] image, int width, int x, int y) { throw new UnsupportedOperationException("Webots API stub"); }
  public static int imageGetGreen(int[] image, int width, int x, int y) { throw new UnsupportedOperationException("Webots API stub"); }
  public static int imageGetBlue(int[] image, int width, int x, int y) { throw new UnsupportedOperationException("Webots API stub"); }
}
