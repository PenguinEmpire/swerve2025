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

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Elevator;
import dev.alphagame.LogManager;

public class ElevatorSubsystem extends SubsystemBase {

    private final SparkMax rightElevatorMotor;
    private final SparkMax leftElevatorMotor;

    // Digital Input for the bottom limit switch


    // The built-in primary (relative) encoder from the Spark Max
    private final RelativeEncoder primaryEncoder;

    private final SparkClosedLoopController pidController;
    private final ArmFeedforward feedforward;

    private double P = 0.3;
    private double I = 0.0;
    private double D = 0.0;
    private double FF = 0.0;
    

    

    private double targetPosition = 0.0;      // Desired setpoint in raw motor revolutions
    private boolean isManualMode  = false;    // Toggle for manual vs. PID mode

    private double newP = 0.0;
    private double newI = 0.0;
    private double newD = 0.0;
    private double newPosition = 0.0;

    public ElevatorSubsystem() {
        // Create the motors
        rightElevatorMotor = new SparkMax(Elevator.RIGHT_ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        leftElevatorMotor  = new SparkMax(Elevator.LEFT_ELEVATOR_MOTOR_ID,  MotorType.kBrushless);
        LogManager.info("Elevator subsystem initialized with motors: R=" + Elevator.RIGHT_ELEVATOR_MOTOR_ID + 
                       ", L=" + Elevator.LEFT_ELEVATOR_MOTOR_ID);

        // Create the DIO limit switch for detecting bottom
       

        // Grab the built-in SparkMax relative encoder 
        primaryEncoder = rightElevatorMotor.getEncoder();

        // Configure right motor:
        SparkMaxConfig rightConfig = new SparkMaxConfig();
        rightConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .closedLoop
                // Use kPrimaryEncoder for the built-in relative encoder
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.85, 0.0, 0.0)  // Adjust these PID gains as needed
                .positionWrappingEnabled(false) // Usually false f∏r an elevator
                .outputRange(-0.4, 0.4);

        
        //  rightConfig.encoder.positionConversionFactor(2.0);

        // Apply configuration
        rightElevatorMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        LogManager.debug("Right elevator motor configured with PID: P=0.85, I=0.0, D=0.0");

        // Configure left motor as a follower of the right motor
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        leftConfig.follow(Elevator.RIGHT_ELEVATOR_MOTOR_ID);
        leftElevatorMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        LogManager.debug("Left elevator motor configured as follower");

        // Access the Spark's built-in closed-loop PID controller
        pidController = rightElevatorMotor.getClosedLoopController();

        // If you need feedforward, set your constants in the ArmFeedforward constructor
        feedforward = new ArmFeedforward(0.0, 0.0, 0.0);

        // Put the manual mode toggle on SmartDashboard
        SmartDashboard.putBoolean("Elevator Manual Mode", false);

        // SmartDashboard.putNumber( "Elevator  P", P);
        // SmartDashboard.putNumber( " ELevator I", I);
        // SmartDashboard.putNumber( " ELevator D", D);

        // SmartDashboard.putNumber()
       
    }

    /** Enables manual (open-loop) mode. */
    public void enableManualMode() {
        isManualMode = true;
        LogManager.info("Elevator manual mode enabled");
        SmartDashboard.putBoolean("Elevator Manual Mode", true);
    }

    /** Enables closed-loop (PID) mode. */
    public void enablePIDMode() {
        isManualMode = false;
        LogManager.info("Elevator PID mode enabled");
        SmartDashboard.putBoolean("Elevator Manual Mode", false);
    }

    /**
     * Manual movement of the elevator.
     * `up = true` drives upwards, `up = false` drives downwards.
     */
    public void manualMove(boolean up) {
        if (isManualMode) { 
            double speed = up ? -Elevator.DEFAULT_ELEVATOR_SPEED : 0.25;
            LogManager.debug("Manual moving elevator " + (up ? "up" : "down") + " with speed: " + speed);
            rightElevatorMotor.set(speed);
        } else {
            LogManager.warning("Attempted manual elevator move while in PID mode");
        }
    }
    /** Stop the elevator immediately (open-loop = 0). */
    public void stopElevator() {
        LogManager.debug("Stopping elevator");
        rightElevatorMotor.set(0);
    }

    /*
     Set a PID position in terms of raw motor revolutions

     */
    public void setPosition(double position) {
        if (!isManualMode) {
            targetPosition = position;
            LogManager.debug("Setting elevator position to: " + position + " revolutions");
            pidController.setReference(targetPosition, ControlType.kPosition);
        } else {
            LogManager.warning("Attempted to set elevator position while in manual mode");
        }
    }

    /**
     * @return The elevator's current position (in raw revolutions),
     *         or in distance units if you used a positionConversionFactor.
     */
    public double getElevatorPosition() {
        return primaryEncoder.getPosition();
    }

    /**
     * Checks whether the elevator is at the bottom limit switch.
     * (Invert if needed, depending on how you wired the switch.)
     */
   
    /**
     * Returns true if the elevator is within "tolerance" of the last commanded target.
     */
    public boolean hasReachedTarget(double tolerance) {
        boolean reached = Math.abs(targetPosition - getElevatorPosition()) <= tolerance;
        if (reached) {
            LogManager.info("Elevator reached target position: " + targetPosition + 
                           " (current: " + getElevatorPosition() + ", tolerance: " + tolerance + ")");
        }
        return reached;
    }

    @Override
    public void periodic() {
        // Check if we changed the manual mode on the SmartDashboard
        boolean previousMode = isManualMode;
        isManualMode = SmartDashboard.getBoolean("Elevator Manual Mode", isManualMode);
        
        if (previousMode != isManualMode) {
            LogManager.info("Elevator mode changed to: " + (isManualMode ? "MANUAL" : "PID"));
        }
        
        // If the bottom limit switch is pressed, zero the relative encoder
       
        // Show data on SmartDashboard for debugging
        SmartDashboard.putNumber("Elevator Position (revs)", getElevatorPosition());
        SmartDashboard.putNumber("Elevator Target (revs)", targetPosition);

     

          
        

    //    double newP  = SmartDashboard.getNumber( "Elevator  P", P);
    //    double newI =  SmartDashboard.getNumber( " ELevator I", I);
    //    double newD =  SmartDashboard.getNumber( " ELevator D", D);
    
    //     if (newP!= P || newI != I || newD!= D) {
    //         P= newP;
    //         I = newI;
    //         velocityGain = newVelocity;
    //         feedforward = new ArmFeedforward(staticGain, gravityGain, velocityGain);
    //     }

        // If we are in manual mode, do not run closed-loop
        if (isManualMode) {
            return;
        }

        // Otherwise, keep commanding PID to the target
        pidController.setReference(targetPosition, ControlType.kPosition);
    }
}
