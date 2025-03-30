package frc.robot.modules;

import com.revrobotics.AbsoluteEncoder;
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


public class Shootermodule {
    private final String name;
    private final SparkMax motor;
    private final SparkClosedLoopController pidController;
    @SuppressWarnings("unused")
    private ArmFeedforward feedforward;
    
    private double targetPosition;
    
    private double armP = 0.3;
    private double armI = 0.0;
    private double armD = 0.0;
    private double armFF = 0.0;

    private double staticGain = 0.0;
    private double gravityGain = 0.0;
    private double velocityGain = 0.0;
   

    public Shootermodule(String name, int motorID) {
        this.name = name + ": ";

        // update motor name 
        motor = new SparkMax(motorID, MotorType.kBrushless);

        // Configure motor
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pid(0.3, 0.0, 0.0)
                .positionWrappingEnabled(true)
                .positionWrappingMinInput(0)
                .positionWrappingMaxInput(2 * Math.PI)
                .outputRange(-1, 1);
        motorConfig.encoder.positionConversionFactor(2 * Math.PI);

        // Apply the initial configuration
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        pidController = motor.getClosedLoopController();

        // Initialize SmartDashboard with PID and FF values
        SmartDashboard.putNumber(name + " P", armP);
        SmartDashboard.putNumber(name + " I", armI);
        SmartDashboard.putNumber(name + " D", armD);
        SmartDashboard.putNumber(name + " FF", armFF);

        SmartDashboard.putNumber(name + " Static Gain", staticGain);
        SmartDashboard.putNumber(name + " Gravity Gain", gravityGain);
        SmartDashboard.putNumber(name + " Velocity Gain", velocityGain);

        // Initialize feedforward
        feedforward = new ArmFeedforward(staticGain, gravityGain, velocityGain);
    }

    /**
     * Sets the target position for the shooter rotation, in radians.
     * @param position The desired position in radians
     */
    public void setPosition(double position) {
        this.targetPosition = position;

        // Send position setpoint to PID controller
        pidController.setReference(targetPosition, SparkBase.ControlType.kPosition);

        // Log reference value for debugging
        SmartDashboard.putNumber(name + " Reference", targetPosition);
    }

    /**
     * Allows manual movement control for testing or calibration.
     * @param speed The motor output from -1.0 to 1.0
     */
    public void manualMove(double speed) {
        motor.set(speed);
    }

    /**
     * Stops the shooter rotation immediately.
     */
    public void stopMotor() {
        motor.set(0);
    }

    /**
     * Gets the current shooter rotation position, in radians.
     * @return The absolute encoder position in radians
     */
    public double getPosition() {
        AbsoluteEncoder absEncoder = motor.getAbsoluteEncoder();
        return absEncoder.getPosition();
    }

    /**
     * Checks if the shooter has reached the target position within a specified tolerance.
     * @param tolerance The allowable error in radians
     * @return True if within tolerance, otherwise false
     */
    public boolean hasReachedTarget(double tolerance) {
        return Math.abs(targetPosition - getPosition()) <= tolerance;
    }

    /**
     * Wraps an angle to always be within [0, 2π] radians.
     * @param _angle The angle to wrap, in radians
     * @return The wrapped angle in [0, 2π]
     */
    public static double WrapAngle(double _angle) {
        double twoPi = 2 * Math.PI;
        if (_angle >= twoPi) {
            return _angle - twoPi * Math.floor(_angle / twoPi);
        } else if (_angle < 0.0) {
            return _angle + twoPi * (Math.floor((-_angle) / twoPi) + 1);
        } else {
            return _angle;
        }
    }

    /**
     * Should be called periodically. Updates dashboard values, checks for changes in feedforward 
     * parameters, and logs important telemetry for debugging.
     */
    public void periodic() {
        // Get the current encoder position
        double currentPos = getPosition();
        
        // Log Encoder Position
        SmartDashboard.putNumber(name + " Encoder Position", currentPos);

        // Log Target Position for Debugging
        SmartDashboard.putNumber(name + " Target Position", targetPosition);

        // Live-update Feedforward Gains from SmartDashboard
        double newStatic = SmartDashboard.getNumber(name + " Static Gain", staticGain);
        double newGravity = SmartDashboard.getNumber(name + " Gravity Gain", gravityGain);
        double newVelocity = SmartDashboard.getNumber(name + " Velocity Gain", velocityGain);

        if (newStatic != staticGain || newGravity != gravityGain || newVelocity != velocityGain) {
            staticGain = newStatic;
            gravityGain = newGravity;
            velocityGain = newVelocity;
            feedforward = new ArmFeedforward(staticGain, gravityGain, velocityGain);
        }
    }
}
