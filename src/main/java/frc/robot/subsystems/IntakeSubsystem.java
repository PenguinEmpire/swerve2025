package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intake;
import frc.robot.modules.Jointmodule;
public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax horizontalRollerMotor;
    private final SparkMax leftVerticalRollerMotor;
    private final SparkMax rightVerticalRollerMotor;
    private final Jointmodule intakeRotation;

  
    private double rollerPower = Intake.DEFAULT_ROLLER_POWER;
    private double rotationPower = Intake.DEFAULT_ROTATION_POWER;
    public IntakeSubsystem() {
        horizontalRollerMotor = new SparkMax(Intake.HORIZONTAL_ROLLER_MOTOR_ID, MotorType.kBrushless);
        leftVerticalRollerMotor = new SparkMax(Intake.LEFT_VERTICAL_ROLLER_MOTOR_ID, MotorType.kBrushless);
        rightVerticalRollerMotor = new SparkMax(Intake.RIGHT_VERTICAL_ROLLER_MOTOR_ID, MotorType.kBrushless);
        intakeRotation = new Jointmodule("Intake Rotation", Intake.ROTATION_MOTOR_ID); 
        

        SmartDashboard.putNumber("Roller Power", Intake.DEFAULT_ROLLER_POWER);
        SmartDashboard.putNumber("Intake Rotation Power", Intake.DEFAULT_ROTATION_POWER);
        SmartDashboard.putNumber("Horizontal Roller Current", horizontalRollerMotor.getOutputCurrent());


        
   
    }

 
    public void spinRollers(boolean intake) {
        // Stop intake if a piece is detected, but allow outtake
        // if (intake && shooterSubsystem.getPiece()) {
        //     stopAllRollers();
        //     return;
        // }
    
        rollerPower = SmartDashboard.getNumber("Roller Power", rollerPower);
        double power = intake ? rollerPower : -rollerPower;
    
        horizontalRollerMotor.set(-power);
        leftVerticalRollerMotor.set( power);
        rightVerticalRollerMotor.set(-power); // Tune this value if needed
    }

    public void stopAllRollers() {
        horizontalRollerMotor.set(0.0);
        leftVerticalRollerMotor.set(0.0);
        rightVerticalRollerMotor.set(0.0); 
    }

    public void setRotationPosition(double position) {
        intakeRotation.setPosition(position); //  Move intake to a target position
    }

    public void manualRotate(boolean down) {
        rotationPower = SmartDashboard.getNumber("Roller Power", rotationPower);
        double power = down ? -rotationPower : rotationPower;  // Move up/down
        intakeRotation.manualMove(power);
    }
    
    public void stopManualRotate() {
        intakeRotation.stopMotor();// Stops rotation when button is released
    }
    
    
    public boolean hasReachedRotationTarget(double tolerance) {
        return intakeRotation.hasReachedTarget(tolerance); //  Check if intake reached position
    }

    // // see if this works
    //  public void checkOvercurrent() {
    //     if (horizontalRollerMotor.getOutputCurrent() > 50) {
    //         stopAllRollers();
    //         new PositionCommand(this, null, PositionCommand.Position.INTAKE_L1).schedule();
    //     }
    // }
    
    @Override
    public void periodic() {
        intakeRotation.periodic();
        
    }
}