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

import dev.alphagame.LogManager;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Elevator;

public class ElevatorSubsystem extends SubsystemBase {

    private final SparkMax rightElevatorMotor;
    private final SparkMax leftElevatorMotor;

    // Built-in primary (relative) encoder from the Spark Max
    private final RelativeEncoder primaryEncoder;
    private final SparkClosedLoopController pidController;
    private final ArmFeedforward feedforward;

    // PID gains
    private double P = 0.3;
    private double I = 0.0;
    private double D = 0.0;
    private double FF = 0.0;

    private double targetPosition = 0.0; // Desired setpoint in raw motor revolutions

    // Elevator speeds read from SmartDashboard (up vs. down)
    private double elevatorSpeed;
    private double elevatorDownSpeed;

    public ElevatorSubsystem() {
        // Create the motors
        rightElevatorMotor = new SparkMax(Elevator.RIGHT_ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        leftElevatorMotor  = new SparkMax(Elevator.LEFT_ELEVATOR_MOTOR_ID,  MotorType.kBrushless);
        
        // Grab the built-in SparkMax relative encoder from the right motor
        primaryEncoder = rightElevatorMotor.getEncoder();

        // Configure right motor
        SparkMaxConfig rightConfig = new SparkMaxConfig();
        rightConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .closedLoop
                // Use kPrimaryEncoder for the built-in relative encoder
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.4, 0.0, 0.0) 
                .positionWrappingEnabled(false)
                .outputRange(-0.4, 0.4);

        rightElevatorMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
       
        // Configure left motor as a follower of the right motor
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        leftConfig.follow(Elevator.RIGHT_ELEVATOR_MOTOR_ID);
        leftElevatorMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Access the Spark's built-in closed-loop PID controller
        pidController = rightElevatorMotor.getClosedLoopController();

        // If you need feedforward, set your constants in the ArmFeedforward constructor
        feedforward = new ArmFeedforward(0.0, 0.0, 0.0);

        // Publish default elevator speeds on the SmartDashboard
        SmartDashboard.putNumber("Elevator Speed", Elevator.DEFAULT_ELEVATOR_SPEED);
        SmartDashboard.putNumber("Elevator Down Speed", Elevator.ELEVATOR_DOWN_SPEED);
    }

    /**
     * Moves the elevator up or down.
     * This is open-loop: up uses "Elevator Speed", down uses "Elevator Down Speed".
     *
     * Since left motor is configured to follow the right motor,
     * we set the right motor. The left motor will automatically match it.
     */
    public void moveElevator(boolean up) {
        // Read the latest speeds from SmartDashboard
        elevatorSpeed = SmartDashboard.getNumber("Elevator Speed", Elevator.DEFAULT_ELEVATOR_SPEED);
        elevatorDownSpeed = SmartDashboard.getNumber("Elevator Down Speed", Elevator.ELEVATOR_DOWN_SPEED);

        // Decide direction/speed
        // If your "up" direction is reversed, you can negate elevatorSpeed.
        double speed = up ? -elevatorSpeed : elevatorDownSpeed;

        // Now drive the right motor (leader).
        rightElevatorMotor.set(speed);
    }

    /** Stop the elevator immediately (open-loop = 0). */
    public void stopElevator() {
        LogManager.debug("Stopping elevator");
        rightElevatorMotor.set(0);
    }

 
    public void setPosition(double position) {
        targetPosition = position;
        LogManager.debug("Setting elevator position to: " + position + " revolutions");
        pidController.setReference(targetPosition, ControlType.kPosition);
    }

   
    public double getElevatorPosition() {
        return primaryEncoder.getPosition();
    }

  
    public boolean hasReachedTarget(double tolerance) {
        boolean reached = Math.abs(targetPosition - getElevatorPosition()) <= tolerance;
        if (reached) {
            LogManager.info("Elevator reached target position: " + targetPosition 
                           + " (current: " + getElevatorPosition() + ", tolerance: " + tolerance + ")");
        }
        return reached;
    }

    @Override
    public void periodic() {
        // Show data on SmartDashboard for debugging
        SmartDashboard.putNumber("Elevator Position (revs)", getElevatorPosition());
        SmartDashboard.putNumber("Elevator Target (revs)", targetPosition);

        // Continually command the PID controller to hold at targetPosition
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
