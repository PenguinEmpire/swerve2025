// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int CONTROLLER_PORT = 0;
    public static final double DEADBAND = 0.05;
    
  }
  
  public static class Intake { //check rotation motor id and also bore encoder, most likely plugged into sparkmax
    public static final int HORIZONTAL_ROLLER_MOTOR_ID = 20;
    public static final int LEFT_VERTICAL_ROLLER_MOTOR_ID = 5;
    public static final int RIGHT_VERTICAL_ROLLER_MOTOR_ID = 1;
    // public static final int ROTATION_MOTOR_ID = 
    
   

 
    public static final double DEFAULT_ROLLER_POWER = 0.45; //tested these values 2/22
    public static final int MOTOR_CURRENT_LIMIT = 30;


}
public static final double MAX_SPEED = Units.feetToMeters(4.5); // potentially change to 14.5
}

