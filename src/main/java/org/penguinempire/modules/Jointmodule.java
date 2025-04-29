package org.penguinempire.modules;

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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.penguinempire.commands.PositionCommand;
/**
 * Controls a robotic joint using a SparkMax motor controller with closed-loop position control.
 * <p>
 * The Jointmodule class provides functionality to control joint positions using both PID feedback
 * and ArmFeedforward for gravity compensation. It's designed for robotic arm joints or similar 
 * rotational mechanisms that require precise position control.
 * <p>
 * Features:
 * <ul>
 *   <li>Closed-loop position control using a SparkMax controller</li>
 *   <li>Absolute encoder position feedback</li>
 *   <li>Position wrapping for continuous rotation joints (0 to 2π radians)</li>
 *   <li>ArmFeedforward integration for gravity compensation</li>
 *   <li>SmartDashboard integration for live tuning of control parameters</li>
 *   <li>Manual control capabilities for testing and calibration</li>
 * </ul>
 * <p>
 * The controller uses both PID for position error correction and feedforward to counteract
 * predictable forces like gravity. Control parameters can be adjusted at runtime through
 * SmartDashboard.
 * 
 * @see ArmFeedforward
 * @see SparkMax
 */
public class Jointmodule {
    private final String name;
    private final SparkMax motor;
    private final SparkClosedLoopController pidController;
    @SuppressWarnings("unused")
    private ArmFeedforward feedforward;
    
    private double targetPosition;
    
    private double UDarmP;
    private double UDarmI;;
    private double UDarmD;;
    private double armFF;

    private double DUarmP;
    private double DUarmI;
    private double DUarmD;

    private double staticGain = 0.0;
    private double gravityGain = 0.0;
    private double velocityGain = 0.0;

    PIDController pidUptoDown = new PIDController(0.2, 0, 0);
    PIDController pidDowntoUp = new PIDController(3.0, 0, 0);
 // two separate pid controllers via the wpilib way not actually configuring it 

    public Jointmodule(String name, int motorID) {
        this.name = name + ": ";
        motor = new SparkMax(3 , MotorType.kBrushless);


        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                //.pid(3.0, 0.0, 0.0)
                .positionWrappingEnabled(true)
                .positionWrappingMinInput(0)
                .positionWrappingMaxInput(2 * Math.PI)
                .outputRange(-1, 1);
        motorConfig.encoder.positionConversionFactor(2 * Math.PI);
        
        // Apply the initial configuration
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        

        pidController = motor.getClosedLoopController();

        // Initialize SmartDashboard with PID and FF values
        SmartDashboard.putNumber(name + " Up -> Down P", UDarmP);
        SmartDashboard.putNumber(name + " Up -> Down I", UDarmI);
        SmartDashboard.putNumber(name + " Up -> Down D", UDarmD);

        SmartDashboard.putNumber(name + " Down -> Up P", DUarmP);
        SmartDashboard.putNumber(name + " Down -> Up I", DUarmI);
        SmartDashboard.putNumber(name + " Down -> Up D", DUarmD);

        SmartDashboard.putNumber(name + " FF", armFF);

        SmartDashboard.putNumber(name + " Static Gain", staticGain);
        SmartDashboard.putNumber(name + " Gravity Gain", gravityGain);
        SmartDashboard.putNumber(name + " Velocity Gain", velocityGain);
        
        // Initialize feedforward
        feedforward = new ArmFeedforward(staticGain, gravityGain, velocityGain);
    
    }
    /**
     * Sets the target position for the joint.
     * @param position
     */
    public void setPosition(double position) {
            this.targetPosition = position;
        
            //  Last year's code only set the target position, without FF here
            //pidController.setReference(targetPosition, SparkBase.ControlType.kPosition);
            if (position == PositionCommand.Position.INTAKE_L1.getEncoderPosition()) { //If the target position is the lower position then it calls pidUptoDown
                motor.set(pidDowntoUp.calculate(motor.getAbsoluteEncoder().getPosition(), targetPosition));
                System.out.println("Down to Up");
            } else if (position == PositionCommand.Position.INTAKE_OUT.getEncoderPosition()) { //If the target position is the upper position then it calls pidDowntoUp
                motor.set(pidUptoDown.calculate(motor.getAbsoluteEncoder().getPosition(), targetPosition));
                System.out.println("Up to Down");
            }

            // Log reference value for debugging
            SmartDashboard.putNumber(name + " Reference", targetPosition);
        
    }

    /**
     * Sets the target position for the joint with feedforward.
     */
    public void manualMove(double speed) {
        motor.set(speed);
    }

    /**
     * Stops the motor immediately.
     */
    public void stopMotor() {
        motor.set(0);
    }

    /**
     * Gets the current position of the joint.
     * @return
     */
    public double getPosition() {
       AbsoluteEncoder absEncoder = motor.getAbsoluteEncoder();

       
      return absEncoder.getPosition();
        
      // return motor.getEncoder().getPosition();
    //   return relativeEncoder.getPosition();
    }

    /**
     * Checks if the joint has reached the target position within a specified tolerance.
     * @param tolerance
     * @return
     */
    public boolean hasReachedTarget(double tolerance) {
        return Math.abs(targetPosition - getPosition()) <= tolerance;
    }

    /**
     * Wraps an angle to always be within [0, 2π] radians.
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

    public void periodic() {
        //  Get the current encoder position
        double currentPos = getPosition();
        
        //  Log Encoder Position
        SmartDashboard.putNumber(name + " Encoder Position", currentPos);
        
        //  Log Target Position for Debugging
        SmartDashboard.putNumber(name + " Target Position", targetPosition);

    
        
        //  Live-update Feedforward Gains from SmartDashboard
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
