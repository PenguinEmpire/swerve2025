package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class PositionCommand extends Command {

    public enum Position {
        // Intake Positions
        INTAKE_IN(0.01),  // Fold intake inside frame
        INTAKE_OUT(0.383), // Extend intake out for pickup

        // Elevator Positions
        ELEVATOR_LOW(0.0),   // figure these values out
        ELEVATOR_LEVEL_1(0.5),  
        ELEVATOR_LEVEL_2(1.0),  
        ELEVATOR_LEVEL_3(1.5),  
        ELEVATOR_MAX(2.0);  

        private final double encoderPosition;

        Position(double encoderPosition) {
            this.encoderPosition = encoderPosition;
        }

        public double getEncoderPosition() {
            return encoderPosition;
        }
    }

    private final IntakeSubsystem intakeSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final Position pos;

    public PositionCommand(IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem, Position pos) {
        this.intakeSubsystem = intakeSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.pos = pos;
        addRequirements(intakeSubsystem, elevatorSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        switch (pos) {
            case INTAKE_IN:
                intakeSubsystem.setRotationPosition(Position.INTAKE_IN.getEncoderPosition());
                break;
            case INTAKE_OUT:
                intakeSubsystem.setRotationPosition(Position.INTAKE_OUT.getEncoderPosition());
                break;
            case ELEVATOR_LOW:
                elevatorSubsystem.setPosition(Position.ELEVATOR_LOW.getEncoderPosition());
                break;
            case ELEVATOR_LEVEL_1:
                elevatorSubsystem.setPosition(Position.ELEVATOR_LEVEL_1.getEncoderPosition());
                break;
            case ELEVATOR_LEVEL_2:
                elevatorSubsystem.setPosition(Position.ELEVATOR_LEVEL_2.getEncoderPosition());
                break;
            case ELEVATOR_LEVEL_3:
                elevatorSubsystem.setPosition(Position.ELEVATOR_LEVEL_3.getEncoderPosition());
                break;
            case ELEVATOR_MAX:
                elevatorSubsystem.setPosition(Position.ELEVATOR_MAX.getEncoderPosition());
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return intakeSubsystem.hasReachedRotationTarget(0.01) || elevatorSubsystem.hasReachedTarget(0.01);
    }
}
