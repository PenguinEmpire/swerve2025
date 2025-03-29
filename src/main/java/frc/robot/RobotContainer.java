// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlignToAprilTag;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here. 
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem();
  // private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  // private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(); 
  // private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  // private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(intakeSubsystem, elevatorSubsystem, climberSubsystem);
  // private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
 
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandPS5Controller m_driverController = 
      new CommandPS5Controller(OperatorConstants.CONTROLLER_PORT);
 
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);

    NamedCommands.registerCommand("AlignToAprilTag", new AlignToAprilTag(drivebase, 1)); //Command for pathplanner which aligns runs aligntoapriltag command
  }
// if this doesnt work swap the x and y
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
    () -> m_driverController.getLeftY()   , // Forward/Backward 
    () -> m_driverController.getLeftX()  ) // Strafe
    .withControllerRotationAxis(() -> m_driverController.getRightX()) // Ensure Rotation is Read
    .deadband(OperatorConstants.DEADBAND)
    .scaleTranslation(0.8)
    .allianceRelativeControl(true); 

                                                                  
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
                                                         .withControllerHeadingAxis(m_driverController::getRightX, m_driverController::getRightY)
                                                         .headingWhile(true);
                                                               
  
  Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);

  Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
  /**
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
        
        // m_driverController.L2()
        // .whileTrue(new RunCommand(() -> {
        //     intakeSubsystem.spinRollers(true);   // Run intake rollers
        //    shooterSubsystem.spinShooter(true);  // Run shooter motor forward for intake
        // }, intakeSubsystem, shooterSubsystem))
        // .onFalse(new InstantCommand(() -> {
        //     intakeSubsystem.stopAllRollers();  // Stop intake rollers
        //     shooterSubsystem.stopShooter();    // Stop shooter motor
        // }, intakeSubsystem, shooterSubsystem));



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
//        m_driverController.L1()
//         .whileTrue(new RunCommand(() -> {
//             intakeSubsystem.spinRollers(false);  // Run outtake rollers
//             shooterSubsystem.spinShooter(false); // Run shooter motor in reverse for outtake
//         }, intakeSubsystem, shooterSubsystem))
//         .onFalse(new InstantCommand(() -> {
//             intakeSubsystem.stopAllRollers();  // Stop intake rollers
//             shooterSubsystem.stopShooter();    // Stop shooter motor
//         }, intakeSubsystem, shooterSubsystem));
// // r2 to shoot piece out
//         m_driverController.R2()
//         .whileTrue(new RunCommand(() -> {
//             shooterSubsystem.spinShooter(false); // Run shooter motor in reverse for outtake
//         },  shooterSubsystem))
//         .onFalse(new InstantCommand(() -> {
//        // Stop intake rollers
//             shooterSubsystem.stopShooter();    // Stop shooter motor
//         }, shooterSubsystem));



    //  pov up → Move Elevator **UP** (While Held)
    // m_driverController.povUp()
    // .whileTrue(new RunCommand(() -> elevatorSubsystem.moveElevator(true), elevatorSubsystem))
    // .onFalse(new InstantCommand(elevatorSubsystem::stopElevator, elevatorSubsystem));

    // // // pov down → Move Elevator **DOWN** (While Held)
    //   m_driverController.povDown()
    //   .whileTrue(new RunCommand(() -> elevatorSubsystem.moveElevator(false), elevatorSubsystem))
    //   .onFalse(new InstantCommand(elevatorSubsystem::stopElevator, elevatorSubsystem));
    
     
      //  POV LEFT → Move Intake **IN**
    //   m_driverController.povLeft()
    //   .onTrue(new PositionCommand(intakeSubsystem, elevatorSubsystem, climberSubsystem, PositionCommand.Position.INTAKE_IN));
  
    //  // POV RIGHT → Move Intake OUT
    //    m_driverController.povRight()
    //   .onTrue(new PositionCommand(intakeSubsystem, elevatorSubsystem, climberSubsystem, PositionCommand.Position.INTAKE_OUT));

//     //  POV UP → Slowly Rotate Intake UP (While Held)
//     m_driverController.povUp()
//       .whileTrue(new RunCommand(() -> intakeSubsystem.moveE(false), intakeSubsystem))
//       .onFalse(new InstantCommand(intakeSubsystem::stopManualRotate, intakeSubsystem));

// //  POV DOWN → Slowly Rotate Intake DOWN (While Held)
//     m_driverController.povDown()
//     .whileTrue(new RunCommand(() -> intakeSubsystem.manualRotate(true), intakeSubsystem))
//     .onFalse(new InstantCommand(intakeSubsystem::stopManualRotate, intakeSubsystem));

// circleButton → Move Elevator to LOW position
 
// // R2 Button → Move Elevator to LEVEL 1 position
//   m_driverController.R2()
//   .onTrue(new PositionCommand(intakeSubsystem, elevatorSubsystem, PositionCommand.Position.ELEVATOR_CRUISING));

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

// // Create Button → Move wacker **UP** (While Held)
//   m_driverController.create()
//   .whileTrue(new RunCommand(() -> climberSubsystem.moveClimber(true), climberSubsystem))
//   .onFalse(new InstantCommand(climberSubsystem::stopClimber, climberSubsystem));
// // Options Button → Move wacker  **DOWN** (While Held)
//   m_driverController.options()
//   .whileTrue(new RunCommand(() -> climberSubsystem.moveClimber(false), climberSubsystem))
//   .onFalse(new InstantCommand(climberSubsystem::stopClimber, climberSubsystem));

// // Create Button → Move wacker to LOW position
// m_driverController.create()
//   .onTrue(new PositionCommand(intakeSubsystem, elevatorSubsystem, climberSubsystem, PositionCommand.Position.CLIMBER_LOW));

// // Options Button → Move wacker to HIGH position
// m_driverController.options()
//   .onTrue(new PositionCommand(intakeSubsystem, elevatorSubsystem, climberSubsystem, PositionCommand.Position.CLIMBER_HIGH));

  // zeros the gyro 
  m_driverController.L3()
    .onTrue(new InstantCommand(drivebase::zeroGyro));


    // m_driverController.R1()
    // .onTrue(new PositionCommand(intakeSubsystem, elevatorSubsystem,climberSubsystem, PositionCommand.Position.ELEVATOR_INTAKEPOS));
  
    // Add a button binding for the AprilTag alignment
    // Using touchpad button for AprilTag alignment with 2.0 degree tolerance
    // keeping it cross for now as all the other systems are non functional
    m_driverController.cross()
      .onTrue(new AlignToAprilTag(drivebase, 1));
 
}




  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Run the MoveForward command in autonomous
    //return drivebase.driveFieldOriented(() -> new ChassisSpeeds(1, 0, 0.0)); <- Old Auto

    return drivebase.getAutonomousCommand("autoTest1");
  }
}