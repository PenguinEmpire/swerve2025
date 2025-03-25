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
import dev.alphagame.LogManager;

public class ClimberSubsystem extends SubsystemBase {
    private final SparkMax climberMotor;
    private final RelativeEncoder climberEncoder;
    private final SparkClosedLoopController pidController;
    private double climberPower = Climber.DEFAULT_CLIMBER_POWER;
    private double targetPosition = 0.0;

    public ClimberSubsystem() {
        climberMotor = new SparkMax(Climber.CLIMBER_MOTOR_ID, MotorType.kBrushless);
        climberEncoder = climberMotor.getEncoder();
        LogManager.info("Climber subsystem initialized with motor ID: " + Climber.CLIMBER_MOTOR_ID);

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
        LogManager.debug("Climber motor configured with PID: P=0.3, I=0.0, D=0.0");
        
        pidController = climberMotor.getClosedLoopController();

        SmartDashboard.putNumber("Climber Power", Climber.DEFAULT_CLIMBER_POWER);
    }

    // Moves climber to a specific position using closed-loop PID
    public void setPosition(double position) {
        targetPosition = position;
        LogManager.debug("Setting climber position to: " + position);
        pidController.setReference(targetPosition, ControlType.kPosition);
    }

    // Checks if the climber has reached the desired position
    public boolean hasReachedTarget(double tolerance) {
        double currentPosition = climberEncoder.getPosition();
        boolean reached = Math.abs(currentPosition - targetPosition) <= tolerance;
        
        if (reached) {
            LogManager.info("Climber reached target position: " + targetPosition + 
                           " (current: " + currentPosition + ", tolerance: " + tolerance + ")");
        }
        
        return reached;
    }

    // Moves climber up or down manually
    public void moveClimber(boolean up) {
        climberPower = SmartDashboard.getNumber("Climber Power", Climber.DEFAULT_CLIMBER_POWER);
        double power = up ? climberPower : -climberPower;
        LogManager.debug("Moving climber " + (up ? "up" : "down") + " with power: " + power);
        climberMotor.set(power);
    }

    // Stops climber movement
    public void stopClimber() {
        LogManager.debug("Stopping climber");
        climberMotor.set(0);
    }

    @Override
    public void periodic() {
        double currentPosition = climberEncoder.getPosition();
        
        // Log if the climber is moving significantly
        if (Math.abs(currentPosition - climberEncoder.getPosition()) > 0.05) {
            LogManager.debug("Climber moving, current position: " + currentPosition);
        }
        
        SmartDashboard.putNumber("Current Climber Power", climberPower);
        SmartDashboard.putNumber("Climber Target Position", targetPosition);
        SmartDashboard.putNumber("Climber Encoder Position", currentPosition);
    }
}
