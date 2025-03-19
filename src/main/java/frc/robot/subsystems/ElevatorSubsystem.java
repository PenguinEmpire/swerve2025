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

public class ElevatorSubsystem extends SubsystemBase {

    private final SparkMax rightElevatorMotor;
    private final SparkMax leftElevatorMotor;

    // Digital Input for the bottom limit switch


    // The built-in primary (relative) encoder from the Spark Max
    private final RelativeEncoder primaryEncoder;

    private final SparkClosedLoopController pidController;
    private final ArmFeedforward feedforward;

    private double targetPosition = 0.0;      // Desired setpoint in raw motor revolutions
    private boolean isManualMode  = false;    // Toggle for manual vs. PID mode

    public ElevatorSubsystem() {
        // Create the motors
        rightElevatorMotor = new SparkMax(Elevator.RIGHT_ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        leftElevatorMotor  = new SparkMax(Elevator.LEFT_ELEVATOR_MOTOR_ID,  MotorType.kBrushless);

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
                .pid(0.1, 0.0, 0.0)  // Adjust these PID gains as needed
                .positionWrappingEnabled(false) // Usually false for an elevator
                .outputRange(-0.6, 0.6);

        
        //  rightConfig.encoder.positionConversionFactor(2.0);

        // Apply configuration
        rightElevatorMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Configure left motor as a follower of the right motor
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        leftConfig.follow(Elevator.RIGHT_ELEVATOR_MOTOR_ID);
        leftElevatorMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Access the Spark's built-in closed-loop PID controller
        pidController = rightElevatorMotor.getClosedLoopController();

        // If you need feedforward, set your constants in the ArmFeedforward constructor
        feedforward = new ArmFeedforward(0.0, 0.3, 0.0);

        // Put the manual mode toggle on SmartDashboard
        SmartDashboard.putBoolean("Elevator Manual Mode", false);
    }

    /** Enables manual (open-loop) mode. */
    public void enableManualMode() {
        isManualMode = true;
        SmartDashboard.putBoolean("Elevator Manual Mode", true);
    }

    /** Enables closed-loop (PID) mode. */
    public void enablePIDMode() {
        isManualMode = false;
        SmartDashboard.putBoolean("Elevator Manual Mode", false);
    }

    /**
     * Manual movement of the elevator.
     * `up = true` drives upwards, `up = false` drives downwards.
     */
    public void manualMove(boolean up) {
        if (isManualMode) { 
            double speed = up ? -Elevator.DEFAULT_ELEVATOR_SPEED : 0.25;
            rightElevatorMotor.set(speed);
        }
    }
    /** Stop the elevator immediately (open-loop = 0). */
    public void stopElevator() {
        rightElevatorMotor.set(0);
    }

    /*
     Set a PID position in terms of raw motor revolutions

     */
    public void setPosition(double position) {
        if (!isManualMode) {
            targetPosition = position;

            // // Optionally add feedforward to your reference.
            // double ffVolts = feedforward.calculate(currentAngle, currentVelocity);
            // pidController.setReference(targetPosition, ControlType.kPosition, 0, ffVolts);

            pidController.setReference(targetPosition, ControlType.kPosition);
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
        return Math.abs(targetPosition - getElevatorPosition()) <= tolerance;
    }

    @Override
    public void periodic() {
        // Check if we changed the manual mode on the SmartDashboard
        isManualMode = SmartDashboard.getBoolean("Elevator Manual Mode", isManualMode);

        // If the bottom limit switch is pressed, zero the relative encoder
       
        // Show data on SmartDashboard for debugging
        SmartDashboard.putNumber("Elevator Position (revs)", getElevatorPosition());
        SmartDashboard.putNumber("Elevator Target (revs)", targetPosition);
        

        // If we are in manual mode, do not run closed-loop
        if (isManualMode) {
            return;
        }

        // Otherwise, keep commanding PID to the target
        pidController.setReference(targetPosition, ControlType.kPosition);
    }
}














// package frc.robot.subsystems;

// import com.revrobotics.AbsoluteEncoder;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.config.SparkMaxConfig;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.Elevator;  
// // Note : elevator down should barely have any input from motor
// public class ElevatorSubsystem extends SubsystemBase {
//     private final SparkMax leftElevatorMotor;
//     private final SparkMax rightElevatorMotor;
//     private final AbsoluteEncoder elevatorEncoder;
    
//     private double elevatorSpeed;
//     private double elevatorDownSpeed;
  

//     public ElevatorSubsystem() {
//         leftElevatorMotor = new SparkMax(Elevator.LEFT_ELEVATOR_MOTOR_ID, MotorType.kBrushless);
//         rightElevatorMotor = new SparkMax(Elevator.RIGHT_ELEVATOR_MOTOR_ID, MotorType.kBrushless);
//         elevatorEncoder = rightElevatorMotor.getAbsoluteEncoder();
        
//         SparkMaxConfig leftConfig = new SparkMaxConfig();
//         leftConfig
//               .inverted(false)  
//               .idleMode(IdleMode.kBrake)  
//               .closedLoop.outputRange(-1.0, 1.0);  

//         SparkMaxConfig rightConfig = new SparkMaxConfig();
//         rightConfig.follow(Elevator.LEFT_ELEVATOR_MOTOR_ID);  


//         leftElevatorMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//         rightElevatorMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

//         elevatorSpeed = Elevator.DEFAULT_ELEVATOR_SPEED;

//         elevatorDownSpeed = Elevator.ELEVATOR_DOWN_SPEED;
    
//         SmartDashboard.putNumber("Elevator Speed", elevatorSpeed);
        
//         SmartDashboard.putNumber("Elevator Down Speed",elevatorDownSpeed);
      
//     }

    

//     // Moves the elevator up or down 
//     public void moveElevator(boolean up) {
//         elevatorSpeed = SmartDashboard.getNumber("Elevator Speed", Elevator.DEFAULT_ELEVATOR_SPEED);
//         elevatorDownSpeed = SmartDashboard.getNumber("Elevator Down Speed", Elevator.ELEVATOR_DOWN_SPEED);
//         double speed = up ? -elevatorSpeed :  0.25;
//         leftElevatorMotor.set(speed);
//     }

//     public double getElevatorPosition() {
//                 return elevatorEncoder.getPosition();
//             }
//     // Stops the elevator movement 
//     public void stopElevator() {
//         leftElevatorMotor.set(0);
//     }

//     @Override
//         public void periodic() {
//             //  Log elevator speed for debugging
//             SmartDashboard.putNumber("Current Elevator Speed", elevatorSpeed);
//             SmartDashboard.putNumber("Current Elevator Down Speed",elevatorDownSpeed );
//             SmartDashboard.putNumber("Elevator Encoder Position", getElevatorPosition());
//         }
// }
