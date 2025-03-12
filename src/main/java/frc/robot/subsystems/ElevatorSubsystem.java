package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
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
    private final SparkClosedLoopController pidController;
    private final AbsoluteEncoder elevatorEncoder;
    private final ArmFeedforward feedforward;

    private double targetPosition = 0.0;
    private boolean isManualMode = false; // Toggle for manual/PID mode

    public ElevatorSubsystem() {
        rightElevatorMotor = new SparkMax(Elevator.RIGHT_ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        leftElevatorMotor = new SparkMax(Elevator.LEFT_ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        elevatorEncoder = rightElevatorMotor.getAbsoluteEncoder();

        // Configure Right Motor (PID & Feedforward)
        SparkMaxConfig rightConfig = new SparkMaxConfig();
        rightConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder) // Use absolute encoder
                .pid(0.1, 0.0, 0.0)  // Adjust PID gains
                .positionWrappingEnabled(false) // No position wrapping for elevator
                .outputRange(-1.0, 1.0); // Allow full power in both directions

             //   rightConfig.encoder.positionConversionFactor(2 * Math.PI);
        
        
        rightElevatorMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Configure Left Motor to Follow Right Motor
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        leftConfig.follow(Elevator.RIGHT_ELEVATOR_MOTOR_ID);  
        leftElevatorMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Initialize PID Controller & Feedforward
        pidController = rightElevatorMotor.getClosedLoopController();
        feedforward = new ArmFeedforward(0.0, 0.0, 0.0); // Adjust gains later

        //  Add SmartDashboard toggle for manual mode
        SmartDashboard.putBoolean("Elevator Manual Mode", false); // Default to PID mode
    }

    //  Toggle Modes from SmartDashboard
    public void enableManualMode() {
        isManualMode = true;
        SmartDashboard.putBoolean("Elevator Manual Mode", true);
    }

    public void enablePIDMode() {
        isManualMode = false;
        SmartDashboard.putBoolean("Elevator Manual Mode", false);
    }

    //  Manual Movement (Up & Down)
    public void manualMove(boolean up) {
        if (isManualMode) { // Only allow manual control in manual mode
            double speed = up ? -Elevator.DEFAULT_ELEVATOR_SPEED : 0.25;
            rightElevatorMotor.set(speed);
        }
    }

    //  Set Position Using PID
    public void setPosition(double position) {
        if (!isManualMode) { // Only run PID if manual mode is disabled
            targetPosition = position;
            pidController.setReference(targetPosition, ControlType.kPosition);
        }
    }

    //  Stop Elevator
    public void stopElevator() {
        rightElevatorMotor.set(0);
    }

    //  Get Encoder Position
    public double getElevatorPosition() {
        return rightElevatorMotor.getAbsoluteEncoder().getPosition();
    }

    //  Check if Elevator Reached Target
    public boolean hasReachedTarget(double tolerance) {
        return Math.abs(targetPosition - getElevatorPosition()) <= tolerance;
    }

    @Override
    public void periodic() {
        //  Read SmartDashboard toggle to check if manual mode is active
        isManualMode = SmartDashboard.getBoolean("Elevator Manual Mode", isManualMode);

        SmartDashboard.putNumber("Elevator Encoder Position", getElevatorPosition());
        SmartDashboard.putNumber("Elevator Target Position", targetPosition);

        //  If manual mode is enabled, do NOT apply PID
        if (isManualMode) return;

        // Apply PID control when not in manual mode
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
