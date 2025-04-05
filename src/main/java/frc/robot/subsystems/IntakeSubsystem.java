package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import dev.alphagame.LogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intake;
import frc.robot.modules.Jointmodule;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax horizontalRollerMotor;
    private final SparkMax leftVerticalRollerMotor;
    private final SparkMax rightVerticalRollerMotor;
     private final SparkMax rollingShooter;
    private final Jointmodule intakeRotation;

  
    private double rollerPower = Intake.DEFAULT_ROLLER_POWER;
    private double rotationPower = Intake.DEFAULT_ROTATION_POWER;
    private double pivotPower = 0.3;
    public IntakeSubsystem() {
        horizontalRollerMotor = new SparkMax(Intake.HORIZONTAL_ROLLER_MOTOR_ID, MotorType.kBrushless);
        leftVerticalRollerMotor = new SparkMax(Intake.LEFT_VERTICAL_ROLLER_MOTOR_ID, MotorType.kBrushless);
        rightVerticalRollerMotor = new SparkMax(Intake.RIGHT_VERTICAL_ROLLER_MOTOR_ID, MotorType.kBrushless);
      rollingShooter = new SparkMax(37,MotorType.kBrushless);
         intakeRotation = new Jointmodule("Intake Rotation", Intake.ROTATION_MOTOR_ID); 
        
        LogManager.info("Intake subsystem initialized with motors: H=" + Intake.HORIZONTAL_ROLLER_MOTOR_ID + 
                        ", LV=" + Intake.LEFT_VERTICAL_ROLLER_MOTOR_ID + 
                        ", RV=" + Intake.RIGHT_VERTICAL_ROLLER_MOTOR_ID + 
                        ", Rotation=" + Intake.ROTATION_MOTOR_ID);

        SmartDashboard.putNumber("Roller Power", Intake.DEFAULT_ROLLER_POWER);
        SmartDashboard.putNumber("Intake Rotation Power", Intake.DEFAULT_ROTATION_POWER);
        SmartDashboard.putNumber("pivot shooter power", pivotPower);
        SmartDashboard.putNumber("Horizontal Roller Current", horizontalRollerMotor.getOutputCurrent());


        
   
    }


    //     // Moves climber up or down manually
    public void movePivot(boolean up) {
       pivotPower = SmartDashboard.getNumber("pivot shooter power", pivotPower);
        double power = up ? pivotPower : - pivotPower;
        LogManager.debug("Moving climber " + (up ? "up" : "down") + " with power: " + power);
        rollingShooter.set(power);
    }

    // // // Stops climber movement
    public void stopPivot() {
        LogManager.debug("Stopping climber");
     rollingShooter.set(0);
    }
 
    public void spinRollers(boolean intake) {
       // Stop intake if a piece is detected, but allow outtake
        // if (intake && shooterSubsystem.getPiece()) {
        //     stopAllRollers();
        //     return;
        // }
    
        rollerPower = SmartDashboard.getNumber("Roller Power", rollerPower);
        double power = intake ? rollerPower : -rollerPower;
    
        LogManager.debug("Spinning intake rollers with power: " + power + " (intake mode: " + intake + ")");
        horizontalRollerMotor.set(-power);
        leftVerticalRollerMotor.set( power);
        rightVerticalRollerMotor.set(-power); // Tune this value if needed
    }

    public void stopAllRollers() {
        LogManager.debug("Stopping all intake rollers");
        horizontalRollerMotor.set(0.0);
        leftVerticalRollerMotor.set(0.0);
        rightVerticalRollerMotor.set(0.0); 
    }

    public void runL1Intake() {
        rollerPower = SmartDashboard.getNumber("Roller Power", rollerPower);
        
        LogManager.debug("Running L1 Intake - horizontal roller intaking, vertical rollers outtaking");
    
        // Horizontal roller intakes
        horizontalRollerMotor.set(- rollerPower);
        // Vertical rollers outtake
        leftVerticalRollerMotor.set(-rollerPower);
        rightVerticalRollerMotor.set(rollerPower); // Inverted direction
    }
    

    public void setRotationPosition(double position) {
        LogManager.debug("Setting intake rotation position to: " + position);
      
        intakeRotation.setPosition(position); //  Move intake to a target position
    }

    public void manualRotate(boolean down) {
        rotationPower = SmartDashboard.getNumber("Roller Power", rotationPower);
        double power = down ? -rotationPower : rotationPower;  // Move up/down
        LogManager.debug("Manual rotating intake: " + (down ? "down" : "up") + " with power: " + power);
        intakeRotation.manualMove(power);
    }
    
    public void stopManualRotate() {
        LogManager.debug("Stopping intake rotation");
        intakeRotation.stopMotor();// Stops rotation when button is released
    }
    
    
    public boolean hasReachedRotationTarget(double tolerance) {
        boolean reached = intakeRotation.hasReachedTarget(tolerance);
        if (reached) {
            LogManager.info("Intake rotation reached target position (tolerance: " + tolerance + ")");
        }
        return reached; //  Check if intake reached position
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
        
        // Monitor motor current for potential jams or issues
        double current = horizontalRollerMotor.getOutputCurrent();
        if (current > 40) {
            LogManager.warning("Intake horizontal roller drawing high current: " + current + "A");
        }
    }
}