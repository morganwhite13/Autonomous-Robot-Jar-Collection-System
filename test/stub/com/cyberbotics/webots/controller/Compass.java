package com.cyberbotics.webots.controller;
public class Compass { public void enable(int t) {} public double[] getValues() { return new double[] {Math.sin(Sim.rad), 0, Math.cos(Sim.rad)}; } }
