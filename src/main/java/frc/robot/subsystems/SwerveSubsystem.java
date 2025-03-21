// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/*
 issues: it cant rotate on its own, it only rotates when you give it front input
 the strafe is completely messed up
 they dont lock back to zero
 not getting dashboard values of encoder 
 should be able to rotate on its own without giving it front and back
 when i press forward, the krakens turn red which means back according to status lights
 when i press back on left joystick it turnms greenw hich means forward according to status light
 when i go left and right strafe is completely messeed up 
 */
package frc.robot.subsystems;

import java.io.File;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

import com.ctre.phoenix6.signals.System_StateValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.studica.frc.AHRS;


public class SwerveSubsystem extends SubsystemBase {
  File directory = new File(Filesystem.getDeployDirectory(),"swerve");
  SwerveDrive  swerveDrive;
  RobotConfig config;
  AHRS navx;
//PiVision vision;

@SuppressWarnings("UseSpecificCatch")
  public SwerveSubsystem() {
  SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try
    {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED,
                                                                  new Pose2d(new Translation2d(Meter.of(1),
                                                                                               Meter.of(4)),
                                                                             Rotation2d.fromDegrees(0)));
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }

    setupPathPlanner();
    /*
    try {
      navx = new AHRS(AHRS.NavXComType.kMXP_SPI);
    } catch (RuntimeException ex) {
      System.err.println("Error instantiating navX: " + ex.getMessage());
    }
      */
  }

  public void setupPathPlanner()
  {
    RobotConfig config;
    try
    {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = true;
      AutoBuilder.configure(
          this::getPose,
          this::resetOdometry,
          // Method to reset odometry (will be called if your auto has a starting pose)
          this::getRobotVelocity,
          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward)
            {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces()
                               );
            } else
            {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
          new PPHolonomicDriveController(
              // PPHolonomicController is the built in path following controller for holonomic drive trains
              new PIDConstants(5.0, 0.0, 0.0),
              // Translation PID constants
              new PIDConstants(5.0, 0.0, 0.0)
              // Rotation PID constants
          ),
          config,
          () -> {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent())
            {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this
        );

    } catch (Exception e)
    {
      e.printStackTrace();
    }

    PathfindingCommand.warmupCommand().schedule();
  }

 @Override
public void periodic() {
  /*
    LimelightHelpers.SetRobotOrientation("limelight", navx.getYaw(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
      
    if(!((Math.abs(navx.getRate()) > 360) || mt2.tagCount == 0)) {
      swerveDrive.addVisionMeasurement(mt2.pose, mt2.timestampSeconds, VecBuilder.fill(.7,.7,9999999));
    }

    */
    for (int i = 0; i < swerveDrive.getModules().length; i++) {
        if (swerveDrive.getModules()[i].getAbsoluteEncoder() != null) {
            double rawValue = swerveDrive.getModules()[i].getAbsoluteEncoder().getAbsolutePosition();
            
            // Change the names slightly to force a refresh ( might not be needed anymore)
            SmartDashboard.putNumber("Module " + i + " Raw Encoder (ABS)", rawValue);
        } else {
            SmartDashboard.putString("Module " + i + " Encoder Status", "Encoder not found!");
        }
    }

    /*
    Pose2d pose = swerveDrive.getPose();
    SmartDashboard.putNumber("PoseX", pose.getX());
    SmartDashboard.putNumber("PoseY", pose.getY());
    */
  }
  


  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  public SwerveDrive getSwerveDrive() {
   return swerveDrive;
  }

/*
public void driveApriltag(double translationX, double translationY, double angularRotationX){
  swerveDrive.drive(new Translation2d(translationX * 1,
                                      translationY * 1),
                                      angularRotationX * 1,
                                      true,
                                      false);
}
*/
  public void drive(double forwardSpeed, double strafeSpeed, double rotationSpeed) {
    SwerveModuleState[] moduleStates = swerveDrive.kinematics.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(forwardSpeed, strafeSpeed, rotationSpeed, swerveDrive.getYaw())
    );
    swerveDrive.setModuleStates(moduleStates, false);
  }



  
  public void driveApriltag(){
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("limelight");

    double id = table.getEntry("tid").getDouble(0);
    double tx = table.getEntry("tx").getDouble(0)-0.2;
    double ty = table.getEntry("ty").getDouble(0)-0.2;

    System.out.println("--------------delin--------------");
    System.out.println(tx);
    System.out.println(ty);

    swerveDrive.drive(new Translation2d(tx,
                                        ty),
                                        0 * 1,
                                        false,
                                        false);

  /*
  Pose2d pose = swerveDrive.getPose();
  double xPose = pose.getX();
  double yPose = pose.getY();
  
  double xTag = 365.20;
  double yTag = 241.64;

  double xMeters = (xPose - xTag)*0.0254;
  double yMeters = (yPose - yTag)*0.0254;

  swerveDrive.drive(new Translation2d(xMeters * 1,
                                      yMeters * 1),
                                      0 * 1,
                                      true,
                                      false);
  */
  }

  public void driveFieldOriented(ChassisSpeeds velocity){
    swerveDrive.driveFieldOriented(velocity);
  }

  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(( ) -> {
      swerveDrive.driveFieldOriented(velocity.get());
    });
  }

  public Pose2d getPose()
  {
    return swerveDrive.getPose();
  }

  public void resetOdometry(Pose2d initialHolonomicPose)
  {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  public ChassisSpeeds getRobotVelocity()
  {
    return swerveDrive.getRobotVelocity();
  }

  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }
  /*
  public Command runAuto(String auto)
  {
    try{
      Command pCommand = new PathPlannerAuto(auto);
      return AutoBuilder.followPath(path);
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      return Commands.none();
    }
  }
  */
  public Command getAutonomousCommand(String pathName)
  {
    return new PathPlannerAuto(pathName);
    //return new PathPlannerAuto(pathName);
  }
}
