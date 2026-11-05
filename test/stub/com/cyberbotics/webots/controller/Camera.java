package com.cyberbotics.webots.controller;
public class Camera { public Camera(String n) {} public void enable(int t) {} public int[] getImage() { return new int[64 * 64]; }
  public static int imageGetRed(int[] i, int w, int x, int y) { return 0; }
  public static int imageGetGreen(int[] i, int w, int x, int y) { return 0; }
  public static int imageGetBlue(int[] i, int w, int x, int y) { return 0; } }
