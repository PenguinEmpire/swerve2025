// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.PositionCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
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
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final SwerveSubsystem drivebase = new SwerveSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
 
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandPS5Controller m_driverController =
      new CommandPS5Controller(OperatorConstants.CONTROLLER_PORT);
 
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
  }

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
    () -> m_driverController.getLeftY(), // Forward/Backward
    () -> m_driverController.getLeftX()) // Strafe
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
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}. 
   */
  private void configureBindings() {
    // Triangle for intake - Runs while button is held
    m_driverController.triangle()
        .whileTrue(new RunCommand(() -> intakeSubsystem.spinRollers(true), intakeSubsystem))
        .onFalse(new InstantCommand(intakeSubsystem::stopAllRollers, intakeSubsystem));

    // Circle for outtake - Runs while button is held
    m_driverController.circle()
        .whileTrue(new RunCommand(() -> intakeSubsystem.spinRollers(false), intakeSubsystem))
        .onFalse(new InstantCommand(intakeSubsystem::stopAllRollers, intakeSubsystem));
    
      //  Square Button → Move Elevator **UP** (While Held)
      m_driverController.square()
      .whileTrue(new RunCommand(() -> elevatorSubsystem.moveElevator(true), elevatorSubsystem))
      .onFalse(new InstantCommand(elevatorSubsystem::stopElevator, elevatorSubsystem));

       //  Cross Button → Move Elevator **DOWN** (While Held)
       m_driverController.cross()
       .whileTrue(new RunCommand(() -> elevatorSubsystem.moveElevator(false), elevatorSubsystem))
       .onFalse(new InstantCommand(elevatorSubsystem::stopElevator, elevatorSubsystem));
    
      //  POV LEFT → Move Intake **IN**
    m_driverController.povLeft()
        .onTrue(new PositionCommand(intakeSubsystem, PositionCommand.Position.INTAKE_IN));

    //  POV RIGHT → Move Intake **OUT**
    m_driverController.povRight()
        .onTrue(new PositionCommand(intakeSubsystem, PositionCommand.Position.INTAKE_OUT));

    //  POV UP → Slowly Rotate Intake UP (While Held)
    m_driverController.povUp()
      .whileTrue(new RunCommand(() -> intakeSubsystem.manualRotate(false), intakeSubsystem))
      .onFalse(new InstantCommand(intakeSubsystem::stopManualRotate, intakeSubsystem));

//  POV DOWN → Slowly Rotate Intake DOWN (While Held)
    m_driverController.povDown()
    .whileTrue(new RunCommand(() -> intakeSubsystem.manualRotate(true), intakeSubsystem))
    .onFalse(new InstantCommand(intakeSubsystem::stopManualRotate, intakeSubsystem));
}


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
} 