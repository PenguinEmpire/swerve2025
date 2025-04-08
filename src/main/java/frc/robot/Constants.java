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
  public static class Logging {
    public static final boolean DO_SWERVE_NOISY_LOGGING = false;
  }
  
  public static class OperatorConstants {
    public static final int CONTROLLER_PORT = 0;
    public static final double DEADBAND = 0.05;
    
  }
  
  public static class Intake { //check rotation motor id and also bore encoder, most likely plugged into sparkmax
    public static final int HORIZONTAL_ROLLER_MOTOR_ID = 54;
    public static final int LEFT_VERTICAL_ROLLER_MOTOR_ID = 2;
    public static final int RIGHT_VERTICAL_ROLLER_MOTOR_ID = 1;
     public static final int ROTATION_MOTOR_ID = 3;
    
   

 
    public static final double DEFAULT_ROLLER_POWER = 1.0; //tested these values 2/22
    public static final double DEFAULT_ROTATION_POWER = 0.05;
    public static final int MOTOR_CURRENT_LIMIT = 30;


}

public static class Elevator {  
  public static final int LEFT_ELEVATOR_MOTOR_ID = 6;  // confirm these tmr
  public static final int RIGHT_ELEVATOR_MOTOR_ID = 17; // the one with the encoder
  public static final double DEFAULT_ELEVATOR_SPEED = 0.3;  // 1 is way too fast 
  public static final double ELEVATOR_DOWN_SPEED = 0.3;
 public static int BOTTOM_LIMIT_SWITCH_CHANNEL = 3;

}

public static class Shooter {  
  public static final int SHOOTER_MOTOR_ID = 22;  // Confirm the ID

  public static final int ALGAE_TOP_MOTOR_ID = 9;

  public static final int ALGAE_BOTTOM_MOTOR_ID = 98;
   public static final int ROTATION_MOTOR_ID = 37;
   public static final double DEFAULT_SHOOTER_POWER = 0.5;
   public static final double DEFAULT_ROTATION_POWER = 0.3;
}

public static class Climber{ // actually using it as the wacker thing
  public static final int CLIMBER_MOTOR_ID = 0;
  public static final double DEFAULT_CLIMBER_POWER = 0.3;
}
public static final double MAX_SPEED = Units.feetToMeters(14.5); // potentially change to 14.5
}

