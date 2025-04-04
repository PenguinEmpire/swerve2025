package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intake;
import frc.robot.modules.Jointmodule;
import dev.alphagame.LogManager;

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

        LogManager.info("Intake subsystem initialized with motors: H=" + Intake.HORIZONTAL_ROLLER_MOTOR_ID +
                ", LV=" + Intake.LEFT_VERTICAL_ROLLER_MOTOR_ID +
                ", RV=" + Intake.RIGHT_VERTICAL_ROLLER_MOTOR_ID +
                ", Rotation=" + Intake.ROTATION_MOTOR_ID);

        SmartDashboard.putNumber("Roller Power", Intake.DEFAULT_ROLLER_POWER);
        SmartDashboard.putNumber("Intake Rotation Power", Intake.DEFAULT_ROTATION_POWER);
        SmartDashboard.putNumber("Horizontal Roller Current", horizontalRollerMotor.getOutputCurrent());

    }

    public void spinRollers(boolean intake) {
        // Stop intake if a piece is detected, but allow outtake
        // if (intake && shooterSubsystem.getPiece()) {
        // stopAllRollers();
        // return;
        // }

        rollerPower = SmartDashboard.getNumber("Roller Power", rollerPower);
        double power = intake ? rollerPower : -rollerPower;

        LogManager.debug("Spinning intake rollers with power: " + power + " (intake mode: " + intake + ")");
        horizontalRollerMotor.set(-power);
        leftVerticalRollerMotor.set(power);
        rightVerticalRollerMotor.set(-power); // Tune this value if needed
    }

    public void stopAllRollers() {
        LogManager.debug("Stopping all intake rollers");
        horizontalRollerMotor.set(0.0);
        leftVerticalRollerMotor.set(0.0);
        rightVerticalRollerMotor.set(0.0);
    }

    public void setRotationPosition(double position) {
        LogManager.debug("Setting intake rotation position to: " + position);
        intakeRotation.setPosition(position); // Move intake to a target position
    }

    public void manualRotate(boolean down) {
        rotationPower = SmartDashboard.getNumber("Roller Power", rotationPower);
        double power = down ? -rotationPower : rotationPower; // Move up/down
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
        return reached; // Check if intake reached position
    }

    // // see if this works
    // public void checkOvercurrent() {
    // if (horizontalRollerMotor.getOutputCurrent() > 50) {
    // stopAllRollers();
    // new PositionCommand(this, null,
    // PositionCommand.Position.INTAKE_L1).schedule();
    // }
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