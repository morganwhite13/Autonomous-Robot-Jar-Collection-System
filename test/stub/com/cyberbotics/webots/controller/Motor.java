package com.cyberbotics.webots.controller;
public class Motor { private final String name; public Motor(String n) { name = n; }
  public void setPosition(double p) {}
  public void setVelocity(double v) { if (name.equals("left wheel")) Sim.vl = v; else if (name.equals("right wheel")) Sim.vr = v; } }
