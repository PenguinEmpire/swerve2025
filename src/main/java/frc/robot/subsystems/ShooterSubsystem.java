package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Shooter;

public class ShooterSubsystem extends SubsystemBase {
    private final SparkMax shooterMotor;
    private final DigitalInput limitSwitch;
    private double shooterPower = Shooter.DEFAULT_SHOOTER_POWER;

    public ShooterSubsystem() {
        shooterMotor = new SparkMax(Shooter.SHOOTER_MOTOR_ID, MotorType.kBrushless);
        limitSwitch = new DigitalInput(9); // Limit switch connected to DIO Port 9

        SmartDashboard.putNumber("Shooter Power", Shooter.DEFAULT_SHOOTER_POWER);
    }

    /** Runs the shooter forward (intake mode) */
    public void spinShooter(boolean intake) {
        shooterPower = SmartDashboard.getNumber("Shooter Power", shooterPower);
    
        //  Stop the shooter if a piece is detected
        if (getPiece()) {
            stopShooter();
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
        boolean pieceDetected = limitSwitch.get(); // Limit switch is active - high (pressed = true) confirm this
        SmartDashboard.putBoolean("Has Piece", pieceDetected);
        return pieceDetected;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Has Piece", getPiece()); // Log on dashboard
    }
}
