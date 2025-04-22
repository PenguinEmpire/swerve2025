// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.penguinempire;

import dev.alphagame.rampage.Rampage;
import dev.alphagame.trailblazer.LogManager;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what
 * you are doing, do not modify this file except to change the parameter class to the startRobot
 * call.
 */
public final class Main {
  private Main() {}

  /**
   * Main initialization function. Do not perform any initialization here.
   *
   * <p>If you change your main robot class, change the parameter type.
   */
  public static void main(String... args) {
    LogManager.info("Starting Robot Program");
    // -- Note --
    // Rampage (dev.alphagame.rampage) is a library that provides a memory monitor and other utilities.
    Rampage.startMemoryMonitor(0.8);
    RobotBase.startRobot(Robot::new);
  }
}
