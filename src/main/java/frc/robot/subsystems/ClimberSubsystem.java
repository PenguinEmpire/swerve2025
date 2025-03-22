// package frc.robot.subsystems;

// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkMax;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.Climber;

// public class ClimberSubsystem extends SubsystemBase {
//     private final SparkMax climberMotor;
//     private double climberPower = Climber.DEFAULT_CLIMBER_POWER;

//     public ClimberSubsystem() {
//         climberMotor = new SparkMax(Climber.CLIMBER_MOTOR_ID, MotorType.kBrushless);

//         SmartDashboard.putNumber("Climber Power", Climber.DEFAULT_CLIMBER_POWER);
//     }

//     // Moves climber up or down
//     public void moveClimber(boolean up) {
//         climberPower = SmartDashboard.getNumber("Climber Power", Climber.DEFAULT_CLIMBER_POWER);
//         double power = up ? climberPower : -climberPower;
//         climberMotor.set(power);
//     }

//     // Stops climber movement
//     public void stopClimber() {
//         climberMotor.set(0);
//     }

//     @Override
//     public void periodic() {
//         SmartDashboard.putNumber("Current Climber Power", climberPower);
//     }
// }



package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Climber;

public class ClimberSubsystem extends SubsystemBase {
    private final SparkMax climberMotor;
    private final RelativeEncoder climberEncoder;
    private final SparkClosedLoopController pidController;
    private double climberPower = Climber.DEFAULT_CLIMBER_POWER;
    private double targetPosition = 0.0;

    public ClimberSubsystem() {
        climberMotor = new SparkMax(Climber.CLIMBER_MOTOR_ID, MotorType.kBrushless);
        climberEncoder = climberMotor.getEncoder();

        // Configure the motor for closed-loop PID control
        SparkMaxConfig climberConfig = new SparkMaxConfig();
        climberConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.3, 0.0, 0.0)  // Adjust these PID gains as needed
                .positionWrappingEnabled(false)
                .outputRange(-0.6, 0.6);

        climberMotor.configure(climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pidController = climberMotor.getClosedLoopController();

        SmartDashboard.putNumber("Climber Power", Climber.DEFAULT_CLIMBER_POWER);
    }

    // Moves climber to a specific position using closed-loop PID
    public void setPosition(double position) {
        targetPosition = position;
        pidController.setReference(targetPosition, ControlType.kPosition);
    }

    // Checks if the climber has reached the desired position
    public boolean hasReachedTarget(double tolerance) {
        double currentPosition = climberEncoder.getPosition();
        return Math.abs(currentPosition - targetPosition) <= tolerance;
    }

    // Moves climber up or down manually
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
        SmartDashboard.putNumber("Climber Target Position", targetPosition);
        SmartDashboard.putNumber("Climber Encoder Position", climberEncoder.getPosition());
    }
}
