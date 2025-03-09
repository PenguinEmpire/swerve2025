package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Climber;

public class ClimberSubsystem extends SubsystemBase {
    private final SparkMax climberMotor;
    private double climberPower = Climber.DEFAULT_CLIMBER_POWER;

    public ClimberSubsystem() {
        climberMotor = new SparkMax(Climber.CLIMBER_MOTOR_ID, MotorType.kBrushless);

        SmartDashboard.putNumber("Climber Power", Climber.DEFAULT_CLIMBER_POWER);
    }

    // Moves climber up or down
    public void moveClimber(boolean up) {
        climberPower = SmartDashboard.getNumber("Climber Power", Climber.DEFAULT_CLIMBER_POWER);
        double power = up ? climberPower : -climberPower;
        climberMotor.set(power);
    }

    // Stops climber movement
    public void stopClimber() {
        climberMotor.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Current Climber Power", climberPower);
    }
}
