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
   //  private final SparkMax rotationMotor;

   // private final DutyCycleEncoder boreEncoder;

   //  private double rotationP = Intake.ROTATION_P;
   //  private double rotationI = Intake.ROTATION_I;
   //  private double rotationD = Intake.ROTATION_D;
    private double rollerPower = Intake.DEFAULT_ROLLER_POWER;

    public IntakeSubsystem() {
        horizontalRollerMotor = new SparkMax(Intake.HORIZONTAL_ROLLER_MOTOR_ID, MotorType.kBrushless);
        leftVerticalRollerMotor = new SparkMax(Intake.LEFT_VERTICAL_ROLLER_MOTOR_ID, MotorType.kBrushless);
        rightVerticalRollerMotor = new SparkMax(Intake.RIGHT_VERTICAL_ROLLER_MOTOR_ID, MotorType.kBrushless);
       // rotationMotor = new SparkMax(Intake.ROTATION_MOTOR_ID, MotorType.kBrushless);

       // boreEncoder = new DutyCycleEncoder(Intake.BORE_ENCODER_CHANNEL);

      //  SmartDashboard.putNumber("Target Angle", 0.0);
        SmartDashboard.putNumber("Roller Power", Intake.DEFAULT_ROLLER_POWER);
     //   SmartDashboard.putNumber("Rotation PID P", rotationP);
    //    SmartDashboard.putNumber("Rotation PID I", rotationI);
     //   SmartDashboard.putNumber("Rotation PID D", rotationD);
    }

   /*   public void rotateToPosition(boolean extended) {
        double targetAngle = extended ? Intake.ROTATION_MAX_ANGLE : 0.0;
        double currentAngle = getArmAngle();

        if (!extended && Math.abs(currentAngle) <= Intake.ROTATION_TOLERANCE) {
            rotationMotor.set(0.0);
            return;
        }

        double error = targetAngle - currentAngle;
        double pidOutput = calculatePID(error);

        rotationMotor.set(pidOutput);
    }

    private double calculatePID(double error) {
        rotationP = SmartDashboard.getNumber("Rotation PID P", rotationP);
        rotationI = SmartDashboard.getNumber("Rotation PID I", rotationI);
        rotationD = SmartDashboard.getNumber("Rotation PID D", rotationD);

        double output = rotationP * error;
        return Math.max(-1.0, Math.min(1.0, output));
    }
*/
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
       // rotationMotor.set(0.0);
    }

   // public double getArmAngle() {
      //  return boreEncoder.get() * 360.0;
  //  }

    @Override
    public void periodic() {
    //    SmartDashboard.putNumber("Current Arm Angle", getArmAngle());
    }
}