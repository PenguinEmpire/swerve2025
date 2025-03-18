package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Shooter;
import frc.robot.commands.PositionCommand;

public class ShooterSubsystem extends SubsystemBase {
    private final SparkMax shooterMotor;
    private final DigitalInput limitSwitch;
    private final IntakeSubsystem intakeSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private double shooterPower = Shooter.DEFAULT_SHOOTER_POWER;

    public ShooterSubsystem(IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        
        shooterMotor = new SparkMax(Shooter.SHOOTER_MOTOR_ID, MotorType.kBrushless);
        limitSwitch = new DigitalInput(9); // Limit switch connected to DIO Port 9

        SmartDashboard.putNumber("Shooter Power", Shooter.DEFAULT_SHOOTER_POWER);
    }

    /** Runs the shooter forward (intake mode) */
    public void spinShooter(boolean intake) {
        shooterPower = SmartDashboard.getNumber("Shooter Power", shooterPower);
        
        // Stop shooter and move robot to position when piece is detected
        if (intake && getPiece()) {
            stopShooter();
            
            intakeSubsystem.stopAllRollers();
            return;
        }
        
        double power = intake ? shooterPower : -shooterPower;
        shooterMotor.set(power);
    }

    /** Stops the shooter */
    public void stopShooter() {
        shooterMotor.set(0.0);
    }

    /** Returns true if the limit switch (piece detection) is pressed */
    public boolean getPiece() {
        boolean pieceDetected = limitSwitch.get(); // Check if the limit switch is triggered
        SmartDashboard.putBoolean("Has Piece", pieceDetected);

        if (pieceDetected) {
            // Stop Intake and Shooter Motors
            stopShooter();

            // Move Elevator to Cruising and Retract Intake
            new SequentialCommandGroup(
                new PositionCommand(intakeSubsystem, elevatorSubsystem, PositionCommand.Position.ELEVATOR_CRUISING),
                new PositionCommand(intakeSubsystem, elevatorSubsystem, PositionCommand.Position.INTAKE_IN)
            ).schedule();
        }

        return pieceDetected;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Has Piece", getPiece()); // Log on dashboard
    }
}
