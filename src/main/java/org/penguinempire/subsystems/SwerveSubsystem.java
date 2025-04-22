// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.penguinempire.subsystems;

import java.io.File;
import java.util.function.Supplier;

import org.penguinempire.Constants;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import dev.alphagame.trailblazer.LogManager;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import static edu.wpi.first.units.Units.Meter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  File directory = new File(Filesystem.getDeployDirectory(), "swerve");
  SwerveDrive swerveDrive;

  @SuppressWarnings("UseSpecificCatch")
  public SwerveSubsystem() {
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    LogManager.info("Initializing Swerve Drive Subsystem");
    try {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED,
          new Pose2d(new Translation2d(Meter.of(1),
              Meter.of(4)),
              Rotation2d.fromDegrees(0)));
      LogManager.info("Swerve Drive initialized successfully with max speed: " + Constants.MAX_SPEED);
      LogManager.warning("Set swerve properties");
      swerveDrive.setHeadingCorrection(true);
      // Alternative method if you don't want to supply the conversion factor via JSON
      // files.
      // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed,
      // angleConversionFactor, driveConversionFactor);

    } catch (Exception e) {
      LogManager.error("Failed to initialize Swerve Drive: " + e.getMessage());
      throw new RuntimeException(e);
    }

    setupPathPlanner();
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
        LogManager.warning("Module " + i + " encoder not found!");
      }
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  private static boolean isZeroVelocity(ChassisSpeeds speeds) {
    return Math.abs(speeds.vxMetersPerSecond) < 0.01 && Math.abs(speeds.vyMetersPerSecond) < 0.01
        && Math.abs(speeds.omegaRadiansPerSecond) < 0.01;
  }

  public void setupPathPlanner() {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = true;
      // Configure AutoBuilder last
      AutoBuilder.configure(
          swerveDrive::getPose,
          // Robot pose supplier
          swerveDrive::resetOdometry,
          // Method to reset odometry (will be called if your auto has a starting pose)
          swerveDrive::getRobotVelocity,
          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward) {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces());
            } else {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also
          // optionally outputs individual module feedforwards
          new PPHolonomicDriveController(
              // probably need to use the heading values instead
              // PPHolonomicController is the built in path following controller for holonomic
              // drive trains
              new PIDConstants(0.01, 0.0, 0.0),
              // Translation PID constants
              new PIDConstants(0.01, 0.0, 0.0)
          // Rotation PID constants
          ),
          config,
          // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red
            // alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this
      // Reference to this subsystem to set requirements
      );

    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Preload PathPlanner Path finding
    // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
    PathfindingCommand.warmupCommand().schedule();
  }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  @SuppressWarnings("unused")
  public void driveFieldOriented(ChassisSpeeds velocity) {
    if (!isZeroVelocity(velocity) && Constants.Logging.DO_SWERVE_NOISY_LOGGING) {
      LogManager.debug("Driving field-oriented: x=" + velocity.vxMetersPerSecond +
          ", y=" + velocity.vyMetersPerSecond +
          ", ω=" + velocity.omegaRadiansPerSecond);
    }
    swerveDrive.driveFieldOriented(velocity);
  }

  public void zeroGyro() {
    LogManager.info("Zeroing gyro");
    swerveDrive.zeroGyro();
  }

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  @SuppressWarnings("unused") // "dead code" raised because of the config being constant (not changing at
                              // runtime)
  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(() -> {
      ChassisSpeeds speeds = velocity.get();

      // only log if we're actually moving

      if (!isZeroVelocity(speeds) && Constants.Logging.DO_SWERVE_NOISY_LOGGING) {
        LogManager.debug("Command driving field-oriented: x=" + speeds.vxMetersPerSecond +
            ", y=" + speeds.vyMetersPerSecond +
            ", w=" + speeds.omegaRadiansPerSecond);
      }
      swerveDrive.driveFieldOriented(speeds);
    });

  }

  public Command getAutonomousCommand(String pathName) {
    // Create a path following command using AutoBuilder. This will also trigger
    // event markers.
    return new PathPlannerAuto(pathName);
  }
}