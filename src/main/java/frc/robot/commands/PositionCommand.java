package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class PositionCommand extends Command {

    public enum Position {
        // Intake Positions
        INTAKE_IN(0.114),  // Fold intake inside frame
        INTAKE_L1(0.00), //figure this value out
        INTAKE_OUT(0.370), // Extend intake out for pickup

        // Climber Positions
        CLIMBER_LOW(0.0),  // Example low position
        CLIMBER_HIGH(10.0), // Example high position

        // Elevator Positions
        ELEVATOR_INTAKEPOS(-6.382),   
        ELEVATOR_CRUISING(-11.142),  
        ELEVATOR_LEVEL_2(-24.523),  
        ELEVATOR_LEVEL_3(-34.880),  
        ELEVATOR_MAX(-53.500),   
        ELEVATOR_ZERO(0.0),

        // shooter Positions

        SHOOTER_INTAKE(0.0),
        SHOOTER_SCORING(1.0);
    
      
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
    private final ClimberSubsystem climberSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final Position pos;

    public PositionCommand(IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem, ClimberSubsystem climberSubsystem, ShooterSubsystem shooterSubsystem,Position pos) {
        this.intakeSubsystem = intakeSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.climberSubsystem = climberSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.pos = pos;
        addRequirements(intakeSubsystem, elevatorSubsystem, climberSubsystem);
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
            case INTAKE_L1:
                 intakeSubsystem.setRotationPosition(Position.INTAKE_L1.getEncoderPosition());
            // case ELEVATOR_INTAKEPOS:
            //     elevatorSubsystem.setPosition(Position.ELEVATOR_INTAKEPOS.getEncoderPosition());
            //     break;
            // case ELEVATOR_CRUISING:
            //     elevatorSubsystem.setPosition(Position.ELEVATOR_CRUISING.getEncoderPosition());
            //     break;
            // case ELEVATOR_LEVEL_2:
            //     elevatorSubsystem.setPosition(Position.ELEVATOR_LEVEL_2.getEncoderPosition());
            //     break;
            // case ELEVATOR_LEVEL_3:
            //     elevatorSubsystem.setPosition(Position.ELEVATOR_LEVEL_3.getEncoderPosition());
            //     break;
            // case ELEVATOR_MAX:
            //     elevatorSubsystem.setPosition(Position.ELEVATOR_MAX.getEncoderPosition());
            //     break;
            // case ELEVATOR_ZERO:
            //     elevatorSubsystem.setPosition(Position.ELEVATOR_ZERO.getEncoderPosition());
          // case SHOOTER_INTAKE:
                // shooterSubsystem.setShooterRotationPosition(Position.SHOOTER_INTAKE.getEncoderPosition());
           
                break;
        }
    }

    @Override
    public boolean isFinished() {
         return intakeSubsystem.hasReachedRotationTarget(0.01);
        //   || elevatorSubsystem.hasReachedTarget(0.01) || shooterSubsystem.hasReachedShooterTarget(0.01);
     
    }
}
