package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Elevator;  

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax leftElevatorMotor;
    private final SparkMax rightElevatorMotor;
    private final AbsoluteEncoder elevatorEncoder;
    private final SparkClosedLoopController pidController;
    private ArmFeedforward feedforward;
    
    private double targetPosition;

    private double elevatorP = 0.3;
    private double elevatorI = 0.0;
    private double elevatorD = 0.0;
    private double elevatorFF = 0.0;

    private double staticGain = 0.0;
    private double gravityGain = 0.0;
    private double velocityGain = 0.0;

    private double elevatorSpeed;
    private double elevatorDownSpeed;

    public ElevatorSubsystem() {
        leftElevatorMotor = new SparkMax(Elevator.LEFT_ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        rightElevatorMotor = new SparkMax(Elevator.RIGHT_ELEVATOR_MOTOR_ID, MotorType.kBrushless);

        // Initialize Absolute Encoder from the right elevator motor
        elevatorEncoder = rightElevatorMotor.getAbsoluteEncoder();

        // Configure left motor with Absolute Encoder & PID
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        leftConfig
            .inverted(false)  
            .idleMode(IdleMode.kBrake)  
            .closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder) //  Configures encoder correctly
                .pid(0.3, 0, 0)
                .positionWrappingEnabled(false)
                .outputRange(-1.0, 1.0);

        //leftConfig.encoder.positionConversionFactor(2 * Math.PI);  could use if needed
        
        leftElevatorMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Make right motor follow left motor **by CAN ID**
        SparkMaxConfig rightConfig = new SparkMaxConfig();
        rightConfig.follow(Elevator.LEFT_ELEVATOR_MOTOR_ID);  //  Correct way
        rightElevatorMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Get PID controller from the left motor
        pidController = leftElevatorMotor.getClosedLoopController();

        
        // Initialize feedforward
        feedforward = new ArmFeedforward(staticGain, gravityGain, velocityGain);

        // Initialize SmartDashboard for PID & FF tuning
        SmartDashboard.putNumber("Elevator P", elevatorP);
        SmartDashboard.putNumber("Elevator I", elevatorI);
        SmartDashboard.putNumber("Elevator D", elevatorD);
        SmartDashboard.putNumber("Elevator FF", elevatorFF);

        SmartDashboard.putNumber("Static Gain", staticGain);
        SmartDashboard.putNumber("Gravity Gain", gravityGain);
        SmartDashboard.putNumber("Velocity Gain", velocityGain);

        elevatorSpeed = Elevator.DEFAULT_ELEVATOR_SPEED;
        elevatorDownSpeed = Elevator.ELEVATOR_DOWN_SPEED;

        SmartDashboard.putNumber("Elevator Speed", elevatorSpeed);
        SmartDashboard.putNumber("Elevator Down Speed", elevatorDownSpeed);
    }

    // Moves the elevator up or down with different speeds (Manual Control)
    public void moveElevator(boolean up) {
        // Get updated values from SmartDashboard
        elevatorSpeed = SmartDashboard.getNumber("Elevator Speed", Elevator.DEFAULT_ELEVATOR_SPEED);
        elevatorDownSpeed = SmartDashboard.getNumber("Elevator Down Speed", Elevator.ELEVATOR_DOWN_SPEED);
        
        // Use different speeds for up/down movement
        double speed = up ? elevatorSpeed : -elevatorDownSpeed;
        leftElevatorMotor.set(speed);
    }

    // Stops the elevator movement (Manual Control)
    public void stopElevator() {
        leftElevatorMotor.set(0);
    }

    // Moves the elevator to a set encoder position (PID Control)
    public void setPosition(double position) {
        this.targetPosition = position;

        // Apply Feedforward
        double ffValue = feedforward.calculate(getElevatorPosition(), 0);
        
        // Apply PID reference with FF
        pidController.setReference(targetPosition, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0, ffValue);
        
        // Log target position
        SmartDashboard.putNumber("Elevator Target Position", targetPosition);
    }

    // Get the elevator's absolute encoder position
    public double getElevatorPosition() {
        return elevatorEncoder.getPosition();
    }

    // Check if elevator reached target position
    public boolean hasReachedTarget(double tolerance) {
        return Math.abs(targetPosition - getElevatorPosition()) <= tolerance;
    }

    @Override
    public void periodic() {
        // Get the current encoder position
        double currentPos = getElevatorPosition();
        
        // Log Encoder Position
        SmartDashboard.putNumber("Elevator Position", currentPos);
        
        // Log Target Position for Debugging
        SmartDashboard.putNumber("Elevator Target Position", targetPosition);

        // Apply PID + FF correction every loop (Only when a target is set)
        if (Math.abs(targetPosition - currentPos) > 0.01) { // Small deadband to prevent jitter
            double ffValue = feedforward.calculate(currentPos, 0);
            pidController.setReference(targetPosition, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0, ffValue);
        }
        
        // Live-update Feedforward Gains from SmartDashboard
        double newStatic = SmartDashboard.getNumber("Static Gain", staticGain);
        double newGravity = SmartDashboard.getNumber("Gravity Gain", gravityGain);
        double newVelocity = SmartDashboard.getNumber("Velocity Gain", velocityGain);

        if (newStatic != staticGain || newGravity != gravityGain || newVelocity != velocityGain) {
            staticGain = newStatic;
            gravityGain = newGravity;
            velocityGain = newVelocity;
            feedforward = new ArmFeedforward(staticGain, gravityGain, velocityGain);
        }
    }
}
