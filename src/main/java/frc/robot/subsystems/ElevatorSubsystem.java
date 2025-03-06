package frc.robot.subsystems;

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
    
    private double elevatorSpeed;
    private double elevatorDownSpeed;

    public ElevatorSubsystem() {
        leftElevatorMotor = new SparkMax(Elevator.LEFT_ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        rightElevatorMotor = new SparkMax(Elevator.RIGHT_ELEVATOR_MOTOR_ID, MotorType.kBrushless);

        
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
        elevatorDownSpeed = Elevator.DOWN_ELEVATOR_SPEED;
        SmartDashboard.putNumber("Elevator Speed", elevatorSpeed);
        SmartDashboard.putNumber("Elevator Down Speed", elevatorDownSpeed);
    }

    // Moves the elevator up or down 
    public void moveElevator(boolean up) {
        elevatorSpeed = SmartDashboard.getNumber("Elevator Speed", Elevator.DEFAULT_ELEVATOR_SPEED);
        elevatorDownSpeed = SmartDashboard.getNumber("Elevator Down Speed", Elevator.DOWN_ELEVATOR_SPEED);
        double speed = up ? elevatorSpeed : - elevatorDownSpeed;
        leftElevatorMotor.set(speed);
    }

    // Stops the elevator movement 
    public void stopElevator() {
        leftElevatorMotor.set(0);
    }

    @Override
    public void periodic() {
        //  Log elevator speed for debugging
        SmartDashboard.putNumber("Current Elevator Speed", elevatorSpeed);
    }
}
