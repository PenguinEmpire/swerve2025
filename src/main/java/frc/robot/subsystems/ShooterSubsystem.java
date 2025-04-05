package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import dev.alphagame.LogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Shooter;
import frc.robot.modules.Shootermodule;

public class ShooterSubsystem extends SubsystemBase {
    private final SparkMax shooterMotor;
    // private final SparkMax algaeTopMotor;

     private final Shootermodule shooterRotation;
    

    private final DigitalInput limitSwitch;
   
    private final IntakeSubsystem intakeSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final ClimberSubsystem climberSubsystem;
    private double shooterPower = Shooter.DEFAULT_SHOOTER_POWER;
   


    public ShooterSubsystem(IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem, ClimberSubsystem climberSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.climberSubsystem = climberSubsystem;

         shooterMotor = new SparkMax(Shooter.SHOOTER_MOTOR_ID, MotorType.kBrushless);

      
         shooterRotation = new Shootermodule(" ShooterRotation", 37); 

        limitSwitch = new DigitalInput(0); // Limit switch connected to DIO Port 0
        LogManager.info("Shooter subsystem initialized with motor ID: " + Shooter.SHOOTER_MOTOR_ID);

        SmartDashboard.putNumber("Shooter Power", Shooter.DEFAULT_SHOOTER_POWER);
    }


    public void setShooterRotationPosition(double position) {
        LogManager.debug("Setting shooter rotation position to: " + position);
        shooterRotation.setPosition(position);
    }
    
    public void manualRotateShooter(boolean down) {
        double rotationPower = SmartDashboard.getNumber("Shooter Rotation Power", 0.3); // default value
        double power = down ? -rotationPower : rotationPower;
        LogManager.debug("Manual rotating shooter: " + (down ? "down" : "up") + " with power: " + power);
        shooterRotation.manualMove(power);
    }
    
    public void stopShooterRotation() {
        LogManager.debug("Stopping shooter rotation");
        shooterRotation.stopMotor();
    }
    
    public boolean hasReachedShooterTarget(double tolerance) {
        boolean reached = shooterRotation.hasReachedTarget(tolerance);
        if (reached) {
            LogManager.info("Shooter rotation reached target position (tolerance: " + tolerance + ")");
        }
        return reached;
    }
    
    /** Runs the shooter forward (intake mode) */
    public void spinShooter(boolean intake) {
        shooterPower = SmartDashboard.getNumber("Shooter Power", shooterPower);
        
        double power = intake ? shooterPower : -shooterPower;
        LogManager.debug("Spinning shooter with power: " + power + " (intake mode: " + intake + ")");
        shooterMotor.set(power);
    }

    /** Stops the shooter */
    public void stopShooter() {
        LogManager.debug("Stopping shooter");
        shooterMotor.set(0.0);
    }


    // public void spinAlgaeShooter(double power) {
    //     LogManager.debug("Spinning algae shooter motors at power: " + power);
    //     algaeTopMotor.set(power);

    // }

    // //  Method to stop both algae shooter motors
    // public void stopAlgaeShooter() {
    //     LogManager.debug("Stopping algae shooter motors");
    //     algaeTopMotor.set(0.0);

    // }


    public boolean getPiece() {
        boolean pieceDetected = limitSwitch.get(); // Check if the limit switch is triggered
        SmartDashboard.putBoolean("Has Piece", pieceDetected);

        if (pieceDetected) {
            LogManager.info("Game piece detected! Stopping shooter and intake");
            // Stop Intake and Shooter Motors
            intakeSubsystem.stopAllRollers();
            stopShooter();

            // Move Elevator to Cruising position and then Retract Intake
            // new SequentialCommandGroup(
            //     new PositionCommand(intakeSubsystem, elevatorSubsystem, climberSubsystem, PositionCommand.Position.ELEVATOR_CRUISING), // Move elevator up first
            //     new PositionCommand(intakeSubsystem, elevatorSubsystem, climberSubsystem, PositionCommand.Position.INTAKE_IN) // Then retract intake
            // ).schedule();
        }

        return pieceDetected;
    }

    @Override
    public void periodic() {
        shooterRotation.periodic();
       boolean hasPiece = getPiece();
   
       SmartDashboard.putBoolean("Has Piece", hasPiece); // Log on dashboard

    }
}
