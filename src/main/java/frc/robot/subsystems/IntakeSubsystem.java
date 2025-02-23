package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intake;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax horizontalRollerMotor;
    private final SparkMax leftVerticalRollerMotor;
    private final SparkMax rightVerticalRollerMotor;
    
  
    private double rollerPower = Intake.DEFAULT_ROLLER_POWER;

    public IntakeSubsystem() {
        horizontalRollerMotor = new SparkMax(Intake.HORIZONTAL_ROLLER_MOTOR_ID, MotorType.kBrushless);
        leftVerticalRollerMotor = new SparkMax(Intake.LEFT_VERTICAL_ROLLER_MOTOR_ID, MotorType.kBrushless);
        rightVerticalRollerMotor = new SparkMax(Intake.RIGHT_VERTICAL_ROLLER_MOTOR_ID, MotorType.kBrushless);
      

        SmartDashboard.putNumber("Roller Power", Intake.DEFAULT_ROLLER_POWER);
   
    }

 
    public void spinRollers(boolean intake) {
        rollerPower = SmartDashboard.getNumber("Roller Power", rollerPower);

        double power = intake ? rollerPower : -rollerPower;
        horizontalRollerMotor.set(-power);
        leftVerticalRollerMotor.set(power);
        rightVerticalRollerMotor.set(-power);
    }

    public void stopAllRollers() {
        horizontalRollerMotor.set(0.0);
        leftVerticalRollerMotor.set(0.0);
        rightVerticalRollerMotor.set(0.0); 
    }

  
    @Override
    public void periodic() {
   
    }
}