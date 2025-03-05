package frc.robot.modules;

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

public class Jointmodule {
    private final String name;
    private final SparkMax motor;
    private final SparkClosedLoopController pidController;
    private ArmFeedforward feedforward;
    
    private double targetPosition;
    
    private double armP = 0.6;
    private double armI = 0.0;
    private double armD = 0.0;
    private double armFF = 0.0;

    private double staticGain = 0.0;
    private double gravityGain = 0.0;
    private double velocityGain = 0.0;

    private SparkMaxConfig storedConfig;  //  Store initial configuration

    public Jointmodule(String name, int motorID) {
        this.name = name + ": ";
        motor = new SparkMax(8 , MotorType.kBrushless);

        //  Initialize and store the initial configuration
        storedConfig = new SparkMaxConfig();
        storedConfig
            .inverted(true)
            .idleMode(IdleMode.kCoast)
            .closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder) // see if this works, it was in absolute before
                .pid(0.1, 0.0, 0.0)
                .positionWrappingEnabled(true) // see if this fixes anything
                .positionWrappingMinInput(0)
                .positionWrappingMaxInput(2 * Math.PI)
                .outputRange(-0.5, 0.5);
        storedConfig.encoder.positionConversionFactor(2 * Math.PI);

        // Apply the initial configuration
        motor.configure(storedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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

        // Initialize target position
        SmartDashboard.putNumber(name + " Target Position", 0.0);
    }

    public void setPosition(double position) {
            this.targetPosition = position;
        
            //  Last year's code only set the target position, without FF here
            pidController.setReference(targetPosition, SparkBase.ControlType.kPosition);
        
            // Log reference value for debugging
            SmartDashboard.putNumber(name + " Reference", targetPosition);
        
    }

    public void manualMove(double speed) {
        motor.set(speed);
    }

    public void stopMotor() {
        motor.set(0);
    }

    public double getPosition() {
        AbsoluteEncoder absEncoder = motor.getAbsoluteEncoder();
        return absEncoder.getPosition();
        
        //return motor.getEncoder().getPosition();
    }

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
        
        //  Read new PID values from SmartDashboard
        double newP = SmartDashboard.getNumber(name + " P", armP);
        double newI = SmartDashboard.getNumber(name + " I", armI);
        double newD = SmartDashboard.getNumber(name + " D", armD);
    
        //  Only update PID if values have changed
        if (newP != armP || newI != armI || newD != armD) {
            SparkMaxConfig newConfig = new SparkMaxConfig();  //  Create a new config object
            newConfig.apply(storedConfig);  //  Apply stored configuration
            newConfig.closedLoop.pid(newP, newI, newD);  //  Modify only PID values
    
            //  Apply only the updated PID settings to the motor
            motor.configure(newConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
            //  Store updated values
            armP = newP;
            armI = newI;
            armD = newD;
        }
        
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
    
        //  Calculate Feedforward using current position
        double ffValue = feedforward.calculate(currentPos, 0);
    
        //  Apply latest reference (PID + FF)
        pidController.setReference(targetPosition, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0, ffValue);
    
        //  Log Feedforward Value
        SmartDashboard.putNumber(name + " Feedforward Value", ffValue);
    }
    
    
}
