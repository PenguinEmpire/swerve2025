

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Elevator;  
// Note : elevator down should barely have any input from motor
public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax leftElevatorMotor;
    private final SparkMax rightElevatorMotor;
    private final AbsoluteEncoder elevatorEncoder;
    
    private double elevatorSpeed;
    private double elevatorDownSpeed;
  

    public ElevatorSubsystem() {
        leftElevatorMotor = new SparkMax(Elevator.LEFT_ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        rightElevatorMotor = new SparkMax(Elevator.RIGHT_ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        elevatorEncoder = rightElevatorMotor.getAbsoluteEncoder();
        
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        leftConfig
              .inverted(false)  
              .idleMode(IdleMode.kBrake)  
              .closedLoop.outputRange(-1.0, 1.0);  

        SparkMaxConfig rightConfig = new SparkMaxConfig();
        rightConfig.follow(Elevator.LEFT_ELEVATOR_MOTOR_ID);  


        leftElevatorMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightElevatorMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        elevatorSpeed = Elevator.DEFAULT_ELEVATOR_SPEED;

        elevatorDownSpeed = Elevator.ELEVATOR_DOWN_SPEED;
    
        SmartDashboard.putNumber("Elevator Speed", elevatorSpeed);
        
        SmartDashboard.putNumber("Elevator Down Speed",elevatorDownSpeed);
      
    }

    // Moves the elevator up or down 
    public void moveElevator(boolean up) {
        elevatorSpeed = SmartDashboard.getNumber("Elevator Speed", Elevator.DEFAULT_ELEVATOR_SPEED);
        elevatorDownSpeed = SmartDashboard.getNumber("Elevator Down Speed", Elevator.ELEVATOR_DOWN_SPEED);
        double speed = up ? -elevatorSpeed :  0.25;
        leftElevatorMotor.set(speed);
    }

    public double getElevatorPosition() {
                return elevatorEncoder.getPosition();
            }
    // Stops the elevator movement 
    public void stopElevator() {
        leftElevatorMotor.set(0);
    }

    @Override
        public void periodic() {
            //  Log elevator speed for debugging
            SmartDashboard.putNumber("Current Elevator Speed", elevatorSpeed);
            SmartDashboard.putNumber("Current Elevator Down Speed",elevatorDownSpeed );
            SmartDashboard.putNumber("Elevator Encoder Position", getElevatorPosition());
        }
}

// package frc.robot.subsystems;

// import com.revrobotics.AbsoluteEncoder;
// import com.revrobotics.spark.SparkBase;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.config.SparkMaxConfig;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.Elevator;

// public class ElevatorSubsystem extends SubsystemBase {
//     // Right motor is the master (has PID and encoder); left motor follows it.
//     private final SparkMax rightElevatorMotor;
//     private final SparkMax leftElevatorMotor;
//     private final AbsoluteEncoder elevatorEncoder;
    
//     // The target position for PID control.
//     private double targetPosition;
    
//     // Speed constants (tunable via SmartDashboard).
//     private double elevatorSpeed;
//     private double elevatorDownSpeed;
    
//     // A scaling factor for manual adjustments (units per call; adjust as needed)
//     private final double manualAdjustmentFactor = 0.05;

//     public ElevatorSubsystem() {
//         // Instantiate motors.
//         rightElevatorMotor = new SparkMax(Elevator.RIGHT_ELEVATOR_MOTOR_ID, MotorType.kBrushless);
//         leftElevatorMotor = new SparkMax(Elevator.LEFT_ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        
//         // Use the absolute encoder from the right (master) motor.
//         elevatorEncoder = rightElevatorMotor.getAbsoluteEncoder();
        
//         // --- Configure the right motor (Master with PID) ---
//         SparkMaxConfig rightConfig = new SparkMaxConfig();
//         rightConfig.inverted(false)
//                    .idleMode(IdleMode.kBrake)
//                    .closedLoop
//                         .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
//                         .pid(1.5, 0.0, 0.0)
//                         // Disable wrapping so the encoder value continues past 2π.
//                         .positionWrappingEnabled(false)
//                         .outputRange(-1.0, 1.0);
//         // Optionally set the conversion factor (e.g. radians).
//         rightConfig.encoder.positionConversionFactor(2 * Math.PI);
//         rightElevatorMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
//         // --- Configure the left motor (Follower) ---
//         SparkMaxConfig leftConfig = new SparkMaxConfig();
//         // Here we call follow on the SparkMaxConfig (using the master’s device ID).
//         leftConfig.follow(Elevator.RIGHT_ELEVATOR_MOTOR_ID);
//         leftElevatorMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
//         // Load speed constants (and allow tuning via SmartDashboard).
//         elevatorSpeed = Elevator.DEFAULT_ELEVATOR_SPEED;
//         elevatorDownSpeed = Elevator.ELEVATOR_DOWN_SPEED;
//         SmartDashboard.putNumber("Elevator Speed", elevatorSpeed);
//         SmartDashboard.putNumber("Elevator Down Speed", elevatorDownSpeed);
        
//         // Initialize the target position to the current encoder reading.
//         targetPosition = getElevatorPosition();
//         SmartDashboard.putNumber("Elevator Target Position", targetPosition);
//     }
    
//     /**
//      * Sets an absolute target position for the elevator.
//      * The right motor’s PID controller drives to this target and the left motor follows.
//      */
//     public void setPosition(double position) {
//         targetPosition = position;
//         rightElevatorMotor.getClosedLoopController()
//             .setReference(targetPosition, SparkBase.ControlType.kPosition);
//         SmartDashboard.putNumber("Elevator Target Position", targetPosition);
//     }
    
//     /**
//      * Moves the elevator up or down at a fixed speed while keeping PID control active.
//      * @param up If true, moves up; otherwise moves down.
//      */
//     public void moveElevator(boolean up) {
//         elevatorSpeed = SmartDashboard.getNumber("Elevator Speed", Elevator.DEFAULT_ELEVATOR_SPEED);
//         elevatorDownSpeed = SmartDashboard.getNumber("Elevator Down Speed", Elevator.ELEVATOR_DOWN_SPEED);
        
//         // Adjust target position by a scaled amount instead of setting percent output.
//         double adjustment = up ? -elevatorSpeed * manualAdjustmentFactor
//                                : elevatorDownSpeed * manualAdjustmentFactor;
//         targetPosition += adjustment;
//         setPosition(targetPosition);
//     }

//     /**
//      * Checks if the elevator has reached its target (within a given tolerance).
//      */
//     public boolean hasReachedTarget(double tolerance) {
//         return Math.abs(getElevatorPosition() - targetPosition) <= tolerance;
//     }
    
//     /**
//      * Returns the continuous encoder position from the right motor.
//      * With wrapping disabled, the value exceeds 2π after one full rotation.
//      */
//     public double getElevatorPosition() {
//         return elevatorEncoder.getPosition();
//     }
    
//     /**
//      * Stops the elevator by setting the target to the current position.
//      */
//     public void stopElevator() {
//         targetPosition = getElevatorPosition();
//         setPosition(targetPosition);
//     }
    
//     @Override
//     public void periodic() {
//         // Update SmartDashboard values.
//         SmartDashboard.putNumber("Elevator Encoder Position", getElevatorPosition());
//         SmartDashboard.putNumber("Elevator Target Position", targetPosition);
        
//         // Optionally, refresh speeds from SmartDashboard.
//         elevatorSpeed = SmartDashboard.getNumber("Elevator Speed", elevatorSpeed);
//         elevatorDownSpeed = SmartDashboard.getNumber("Elevator Down Speed", elevatorDownSpeed);
//     }
// }
