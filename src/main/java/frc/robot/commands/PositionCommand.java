package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class PositionCommand extends Command {

    public enum Position {
        INTAKE_IN(0.0),  // Fold intake inside frame
        INTAKE_OUT(3.91); // Extend intake out for pickup ( find this value from testing)

        private final double encoderPosition;

        Position(double encoderPosition) {
            this.encoderPosition = encoderPosition;
        }

        public double getEncoderPosition() {
            return encoderPosition;
        }
    }

    private final IntakeSubsystem intakeSubsystem;
    private final Position pos;
    private int m_ticks = 0;

    public PositionCommand(IntakeSubsystem intakeSubsystem, Position pos) {
        this.intakeSubsystem = intakeSubsystem;
        this.pos = pos;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        m_ticks = 0;
    }

    @Override
    public void execute() {
        m_ticks++;

        if (pos == Position.INTAKE_IN) {
            intakeSubsystem.setRotationPosition(Position.INTAKE_IN.getEncoderPosition());
        } else if (pos == Position.INTAKE_OUT) {
            intakeSubsystem.setRotationPosition(Position.INTAKE_OUT.getEncoderPosition());
        }
    }

    @Override
    public boolean isFinished() {
        return m_ticks > 20; // Runs for a short time to ensure the position is set
    }
}
