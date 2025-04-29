package org.penguinempire.commands;

import org.penguinempire.subsystems.ElevatorSubsystem;
import org.penguinempire.subsystems.IntakeSubsystem;
import org.penguinempire.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.Command;


public class PositionCommand extends Command {

    public enum Position {
        // Intake Positions
        INTAKE_IN(0.0),  // Fold intake inside frame
        INTAKE_L1(0.985), //figure this value out
        INTAKE_OUT(0.692), // Extend intake out for pickup origionally 0.648

        // Climber Positions
        CLIMBER_LOW(0.0),  // Example low position
        CLIMBER_HIGH(10.0), // Example high position

        // Elevator Positions
        ELEVATOR_INTAKEPOS(0),   
        ELEVATOR_CRUISING(-11.142),  
        ELEVATOR_LEVEL_2(-4.4357),  
        ELEVATOR_LEVEL_3(-7.190),  
        ELEVATOR_MAX(-24.690),   
        ELEVATOR_ZERO(0.0),

        // shooter Positions

        SHOOTER_INTAKE(0.0),
        SHOOTER_SCORING(0.190),
        SHOOTER_l2(0.344),
        SHOOTER_l3(0.279),
        SHOOTER_TEST(0.178);
    
      
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
    // private final ClimberSubsystem climberSubsystem;
     private final ShooterSubsystem shooterSubsystem;
         private final Position pos;
     
         public PositionCommand(IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem, ShooterSubsystem shooterSubsystem,Position pos) {
         //, ElevatorSubsystem elevatorSubsystem, ClimberSubsystem climberSubsystem, ShooterSubsystem shooterSubsystem,Position pos) {
             this.intakeSubsystem = intakeSubsystem;
              this.elevatorSubsystem = elevatorSubsystem;
             // this.climberSubsystem = climberSubsystem;
          this.shooterSubsystem = shooterSubsystem;
        this.pos = pos;
        addRequirements(intakeSubsystem, elevatorSubsystem,shooterSubsystem);
        // , elevatorSubsystem, climberSubsystem);
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
            case ELEVATOR_INTAKEPOS:
                elevatorSubsystem.setPosition(Position.ELEVATOR_INTAKEPOS.getEncoderPosition());
                break;
            case ELEVATOR_CRUISING:
                elevatorSubsystem.setPosition(Position.ELEVATOR_CRUISING.getEncoderPosition());
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
            case ELEVATOR_ZERO:
                elevatorSubsystem.setPosition(Position.ELEVATOR_ZERO.getEncoderPosition());
           case SHOOTER_INTAKE:
                 shooterSubsystem.setShooterRotationPosition(Position.SHOOTER_INTAKE.getEncoderPosition());
            case SHOOTER_l2:
                  shooterSubsystem.setShooterRotationPosition(Position.SHOOTER_l2.getEncoderPosition());
                break;
            case SHOOTER_TEST:
            shooterSubsystem.setShooterRotationPosition(Position.SHOOTER_TEST.getEncoderPosition());
                break;
            default:
                // Handle other positions or do nothing
                break;
            
        }
    }

    @Override
    public boolean isFinished() {
         return intakeSubsystem.hasReachedRotationTarget(0.01)
         || elevatorSubsystem.hasReachedTarget(0.01)
         || shooterSubsystem.hasReachedShooterTarget(0.01);
     
    }
}
