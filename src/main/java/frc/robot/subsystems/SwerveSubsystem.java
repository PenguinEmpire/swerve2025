// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import static edu.wpi.first.units.Units.Meter;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;



public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

File directory = new File(Filesystem.getDeployDirectory(),"swerve");
SwerveDrive  swerveDrive;
@SuppressWarnings("UseSpecificCatch")
  public SwerveSubsystem() {
  SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
      try
    {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED,
                                                                  new Pose2d(new Translation2d(Meter.of(1),
                                                                                               Meter.of(4)),
                                                                             Rotation2d.fromDegrees(0)));
      // Alternative method if you don't want to supply the conversion factor via JSON files.
      // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);

    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }


  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

 @Override
public void periodic() {
    for (int i = 0; i < swerveDrive.getModules().length; i++) {
        if (swerveDrive.getModules()[i].getAbsoluteEncoder() != null) {
            double rawValue = swerveDrive.getModules()[i].getAbsoluteEncoder().getAbsolutePosition();
            
            // Change the names slightly to force a refresh ( might not be needed anymore)
            SmartDashboard.putNumber("Module " + i + " Raw Encoder (ABS)", rawValue);
        } else {
            SmartDashboard.putString("Module " + i + " Encoder Status", "Encoder not found!");
        }
    }
  }
  


  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  public SwerveDrive getSwerveDrive() {
   return swerveDrive;
  }

public void driveFieldOriented(ChassisSpeeds velocity){
  swerveDrive.driveFieldOriented(velocity);
}

public void zeroGyro() {
  swerveDrive.zeroGyro();
}

public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
  return run(( ) -> {
     swerveDrive.driveFieldOriented(velocity.get());
  });
}


}
