// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.penguinempire;

import org.penguinempire.Constants.OperatorConstants;
import org.penguinempire.commands.PositionCommand;
import org.penguinempire.commands.SwerveDriveCommand;
import org.penguinempire.subsystems.ClimberSubsystem;
import org.penguinempire.subsystems.ElevatorSubsystem;
import org.penguinempire.subsystems.IntakeSubsystem;
import org.penguinempire.subsystems.ShooterSubsystem;
import org.penguinempire.subsystems.SwerveSubsystem;

// import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import swervelib.SwerveInputStream;
;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here. 
 */
public class RobotContainer {
  private final SendableChooser<String> autoChooser = new SendableChooser<>();
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem();
   private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
   private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(); 
   private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(intakeSubsystem,  elevatorSubsystem,climberSubsystem);
   //private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
 
  BooleanReference L3Pressed = new BooleanReference(false);
  BooleanReference PsPressed = new BooleanReference(false);
  BooleanReference R3Pressed = new BooleanReference(false);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandPS5Controller m_driverController = 
      new CommandPS5Controller(OperatorConstants.CONTROLLER_PORT);
 
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    drivebase.setDefaultCommand(new SwerveDriveCommand(drivebase, driveAngularVelocity, L3Pressed, PsPressed, R3Pressed));
    //drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    autoChooser.addOption("FRC Auto 1", "FRCAuto");
    autoChooser.addOption("Straight Line Auto", "StraightLineAuto");
    autoChooser.addOption("Big Spin Auto", "BigSpinAuto");
    autoChooser.addOption("Little Spin Auto", "LittleSpinAuto");

    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putBoolean("LOffset", false);
    SmartDashboard.putBoolean("ROffset", false);

  }
// if this doesnt work swap the x and y

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
    () -> m_driverController.getLeftY() * -1  , // Forward/Backward 
    () -> m_driverController.getLeftX()  * -1 ) // Strafe
    .withControllerRotationAxis(() -> m_driverController.getRightX() * -1) // Ensure Rotation is Read
    .deadband(OperatorConstants.DEADBAND)
    .scaleTranslation(0.8)
    .allianceRelativeControl(true); 

                                                                   
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
                                                         .withControllerHeadingAxis(m_driverController::getRightX, m_driverController::getRightY)
                                                         .headingWhile(true);
                                                                
  Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);

  Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

  /*
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link     
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller>
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}. 
   */
  private void configureBindings() {
        // l2 for intake - Runs while button is held
        
        m_driverController.triangle()
        .whileTrue(new RunCommand(() -> {
            intakeSubsystem.spinRollers(true);   // Run intake rollers
           shooterSubsystem.spinShooter(true);  // Run shooter motor forward for intake
        }, intakeSubsystem, shooterSubsystem))
        .onFalse(new InstantCommand(() -> {
            intakeSubsystem.stopAllRollers();  // Stop intake rollers
            shooterSubsystem.stopShooter();    // Stop shooter motor
        }, intakeSubsystem, shooterSubsystem));

        // m_driverController.L2()
        // .whileTrue(new RunCommand(() -> {
        //     intakeSubsystem.spinRollers(true);   // Run intake rollers
        // }, intakeSubsystem))
        // .onFalse(new InstantCommand(() -> {
        //     intakeSubsystem.stopAllRollers();  // Stop intake rollers
        // }, intakeSubsystem));


    // m_driverController.L2()
    // .onTrue(
    //     new SequentialCommandGroup(
    
    //       // new PositionCommand(intakeSubsystem, elevatorSubsystem,climberSubsystem, PositionCommand.Position.ELEVATOR_CRUISING),
    //         // Step 1: Move the intake down first

    //         new PositionCommand(intakeSubsystem, elevatorSubsystem, climberSubsystem, PositionCommand.Position.INTAKE_OUT),
    //         // Step 2: After intake is down, move the elevator
    //         new PositionCommand(intakeSubsystem, elevatorSubsystem, climberSubsystem, PositionCommand.Position.ELEVATOR_INTAKEPOS),
    //         // Step 3: Now that intake & elevator are in position, start intake rollers and shooter rollers
    //         new RunCommand(() -> {
    //             intakeSubsystem.spinRollers(true);
    //             shooterSubsystem.spinShooter(true);
    //         }, intakeSubsystem, shooterSubsystem)
    //         .until(() -> shooterSubsystem.getPiece()) // Stop when the limit switch is triggered
    //     )
        
    // );

    //l1 for outtake - Runs while button is held
       m_driverController.cross()
        .whileTrue(new RunCommand(() -> {
            intakeSubsystem.spinRollers(false);  // Run outtake rollers
             shooterSubsystem.spinShooter(false); // Run shooter motor in reverse for outtake
        }, intakeSubsystem, shooterSubsystem))
        .onFalse(new InstantCommand(() -> {
            intakeSubsystem.stopAllRollers();  // Stop intake rollers
             shooterSubsystem.stopShooter();    // Stop shooter motor
        }, intakeSubsystem, shooterSubsystem));

      m_driverController.R1()
        .whileTrue(new RunCommand(() -> {
            intakeSubsystem.spinRollers(false);  // Run outtake rollers
        }, intakeSubsystem))
        .onFalse(new InstantCommand(() -> {
            intakeSubsystem.stopAllRollers();  // Stop intake rollers
        }, intakeSubsystem));

// // r2 to shoot piece out
//         m_driverController.R2()
//         .whileTrue(new RunCommand(() -> {
//             shooterSubsystem.spinShooter(false); // Run shooter motor in reverse for outtake
//         },  shooterSubsystem))
//         .onFalse(new InstantCommand(() -> {
//        // Stop intake rollers
//             shooterSubsystem.stopShooter();    // Stop shooter motor
//         }, shooterSubsystem));




    // //  pov up → Move Elevator **UP** (While Held)
    // m_driverController.povLeft() 
    // .whileTrue(new RunCommand(() -> elevatorSubsystem.moveElevator(true), elevatorSubsystem))
    // .onFalse(new InstantCommand(elevatorSubsystem::stopElevator, elevatorSubsystem));

    // // // // // // pov down → Move Elevator **DOWN** (While Held)
    //   m_driverController.povRight()
    //   .whileTrue(new RunCommand(() -> elevatorSubsystem.moveElevator(false), elevatorSubsystem))
    //   .onFalse(new InstantCommand(elevatorSubsystem::stopElevator, elevatorSubsystem));
    
     
   
// // // // R2 Button → Move Elevator to LEVEL 2 position
//   m_driverController.povUp()
//   .onTrue(new PositionCommand(intakeSubsystem, elevatorSubsystem,shooterSubsystem,PositionCommand.Position.SHOOTER_TEST));

  // m_driverController.circle()
  //   .onTrue(
  //       new SequentialCommandGroup(
  //           new PositionCommand(intakeSubsystem, elevatorSubsystem,climberSubsystem, PositionCommand.Position.ELEVATOR_CRUISING), // Step 1: Move Elevator to Cruising Position
  //           new PositionCommand(intakeSubsystem, elevatorSubsystem, climberSubsystem, PositionCommand.Position.INTAKE_IN) // Step 2: Retract Intake
  //       )
  //   );

// cross Button → Move Elevator to LEVEL 2 position
//   m_driverController.cross()
//   .onTrue(new PositionCommand(intakeSubsystem, elevatorSubsystem,climberSubsystem, PositionCommand.Position.ELEVATOR_LEVEL_2));

//   m_driverController.circle()
//   .onTrue(new PositionCommand(intakeSubsystem, elevatorSubsystem,climberSubsystem, PositionCommand.Position.ELEVATOR_CRUISING));

// // R1 Button → Move Elevator to LEVEL 3 position
//   m_driverController.square()
//   .onTrue(new PositionCommand(intakeSubsystem, elevatorSubsystem, climberSubsystem, PositionCommand.Position.ELEVATOR_LEVEL_3));

// // R1 Button → Move Elevator to LEVEL 4 position
// m_driverController.triangle()
// .onTrue(new PositionCommand(intakeSubsystem, elevatorSubsystem, climberSubsystem, PositionCommand.Position.ELEVATOR_MAX));  

 

// Create Button → Move climber UP
m_driverController.create()
    .onTrue(new InstantCommand(() -> climberSubsystem.moveUp(), climberSubsystem))
    .onFalse(new InstantCommand(() -> climberSubsystem.stop(), climberSubsystem));

// Options Button → Move climber DOWN
m_driverController.options()
    .onTrue(new InstantCommand(() -> climberSubsystem.moveDown(), climberSubsystem))
    .onFalse(new InstantCommand(() -> climberSubsystem.stop(), climberSubsystem));


  // zeros the gyro 
  // m_driverController.cross()
  // .onTrue(new InstantCommand(drivebase::zeroGyro));


    // m_driverController.R1()
    // .onTrue(new PositionCommand(intakeSubsystem, elevatorSubsystem,climberSubsystem, PositionCommand.Position.ELEVATOR_INTAKEPOS));
  
    // Add a button binding for the AprilTag alignment
    // Using touchpad button for AprilTag alignment with 2.0 degree tolerance
    // keeping it cross for now as all the other systems are non functional

    m_driverController.L3()
      .whileTrue(new InstantCommand(() -> {
        L3Pressed.bool = true;
      }))
      .onFalse(new InstantCommand(() -> {
        L3Pressed.bool = false;
      }));

      m_driverController.PS()
      .whileTrue(new InstantCommand(() -> {
        PsPressed.bool = true;
      }))
      .onFalse(new InstantCommand(() -> {
        PsPressed.bool = false;
      }));

      m_driverController.R3()
      .whileTrue(new InstantCommand(() -> {
        R3Pressed.bool = true;
      }))
      .onFalse(new InstantCommand(() -> {
        R3Pressed.bool = false;
      }));

 // Square → Manual Control of Shooter Pivot (Hold = Move Up) ( make it false to spin the other way)
m_driverController.square()
  .whileTrue(new RunCommand(() -> shooterSubsystem.manualRotateShooter(false), shooterSubsystem))
  .onFalse(new InstantCommand(shooterSubsystem::stopShooterRotation, shooterSubsystem));

  m_driverController.circle()
  .whileTrue(new RunCommand(() -> shooterSubsystem.manualRotateShooter(true), shooterSubsystem))
  .onFalse(new InstantCommand(shooterSubsystem::stopShooterRotation, shooterSubsystem));


//   m_driverController.square()
//     .onTrue(new InstantCommand(() -> intakeSubsystem.movePivot(true), intakeSubsystem))
//     .onFalse(new InstantCommand(() -> intakeSubsystem.stopPivot(), intakeSubsystem));

// // // Options Button → Move climber DOWN
// m_driverController.circle()
// .onTrue(new InstantCommand(() -> intakeSubsystem.movePivot(false), intakeSubsystem))
// .onFalse(new InstantCommand(() -> intakeSubsystem.stopPivot(), intakeSubsystem));


// binds to run the algae motors forward and reverse
// m_driverController.triangle()
//     .whileTrue(new RunCommand(() -> shooterSubsystem.spinAlgaeShooter(0.5), shooterSubsystem))  // Adjust power if needed
//     .onFalse(new InstantCommand(() -> shooterSubsystem.stopAlgaeShooter(), shooterSubsystem));

// // Circle → Run Algae Motors REVERSE while held
m_driverController.povUp()
    .whileTrue(new RunCommand(() -> shooterSubsystem.spinAlgaeShooter(-0.5), shooterSubsystem)) // Reverse power
    .onFalse(new InstantCommand(() -> shooterSubsystem.stopAlgaeShooter(), shooterSubsystem));

// binds for l1 intake mode to purposely jam the piece

m_driverController.L1()
  .whileTrue(new RunCommand(() -> {
      intakeSubsystem.runL1Intake();  // Run L1 intake mode (horizontal in, vertical out)
  }, intakeSubsystem))
  .onFalse(new InstantCommand(() -> {
      intakeSubsystem.stopAllRollers();  // Stop rollers on release
  }, intakeSubsystem));

  // bind to test l1 position
  m_driverController.R2()
  .onTrue(new PositionCommand(intakeSubsystem,elevatorSubsystem, shooterSubsystem,PositionCommand.Position.INTAKE_L1));

  m_driverController.L2()
  .onTrue(new PositionCommand(intakeSubsystem,elevatorSubsystem,shooterSubsystem, PositionCommand.Position.INTAKE_OUT));

}





  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Run the MoveForward command in autonomous
    //return drivebase.driveFieldOriented(() -> new ChassisSpeeds(1, 0, 0.0)); <- Old Auto

    return drivebase.getAutonomousCommand(autoChooser.getSelected());
  }
}