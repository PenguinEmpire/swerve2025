package frc.robot.modules;

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
    private final ArmFeedforward feedforward;
    
    private double targetPosition;

  

    public Jointmodule(String name, int motorID) {
        this.name = name + ": ";
        motor = new SparkMax(motorID, MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
       // pidController = motor.getClosedLoopController();

        // Configure motor settings
        
        config
            .inverted(true)
            .idleMode(IdleMode.kBrake);

     
            config.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder) // Use absolute encoder mode
            .pid(1.0, 0.0, 0.0) // Default PID values
            .positionWrappingEnabled(true)  
            .positionWrappingMinInput(0)   
            .positionWrappingMaxInput(2 * Math.PI)  
            .outputRange(-0.2, 0.2); // increase after testing
    

        
        
            config.encoder
            .positionConversionFactor(2 * Math.PI);  // Convert encoder values to radians
            
        

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

         pidController = motor.getClosedLoopController();
        
         SmartDashboard.putNumber(name + " P", 1.0);
         SmartDashboard.putNumber(name + " I", 0.0);
         SmartDashboard.putNumber(name + " D", 0.0);
         SmartDashboard.putNumber(name + " FF", 0.0);
        // Setup Feedforward for gravity compensation
        feedforward = new ArmFeedforward(0.1, 0.4, 0.0);

        // Initialize SmartDashboard for tuning
        SmartDashboard.putNumber(name + " Target Position", 0.0);
    }

    public void setPosition(double position) {
        
        this.targetPosition = position;
        double ffValue = feedforward.calculate(targetPosition, 0);
        pidController.setReference(targetPosition, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0, ffValue);
        SmartDashboard.putNumber(name + " Target Position", targetPosition);
    }

    public void manualMove(double speed) {
        motor.set(speed);
    }

    public void stopMotor() {
        motor.set(0);
    }

    public double getPosition() {
        return motor.getEncoder().getPosition();

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
    //  Log Encoder Position
    SmartDashboard.putNumber(name + " Encoder Position", getPosition());

    //  Log Target Position for Debugging
    SmartDashboard.putNumber(name + " Target Position", targetPosition);

    //  Log Feedforward Components
    // SmartDashboard.putNumber(name + " Static Gain", feedforward.ks);
    // SmartDashboard.putNumber(name + " Gravity Gain", feedforward.kg);
    // SmartDashboard.putNumber(name + " Velocity Gain", feedforward.kv);
}

}

