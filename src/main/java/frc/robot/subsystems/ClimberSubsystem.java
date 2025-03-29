package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Climber;

public class ClimberSubsystem extends SubsystemBase {
    private final TalonFX climberMotor;



    public ClimberSubsystem() {
        // Instantiate the TalonFX device
        climberMotor = new TalonFX(Climber.CLIMBER_MOTOR_ID);

        //  Factory default  then configure
        TalonFXConfiguration initialFactoryDefault = new TalonFXConfiguration();
        StatusCode factoryDefaultStatus = climberMotor.getConfigurator().apply(initialFactoryDefault);
        if (!factoryDefaultStatus.isOK()) {
            System.err.println("Could not factory-default TalonFX: " + factoryDefaultStatus);
        }

        //  Create a TalonFXConfiguration for permanent settings
        TalonFXConfiguration talonConfig = new TalonFXConfiguration();

        // Configure motor output settings
        MotorOutputConfigs motorOutput = new MotorOutputConfigs();
        // Brake or Coast for neutral
        motorOutput.NeutralMode = NeutralModeValue.Brake;
        // Choose the direction for "positive" outputs
        motorOutput.Inverted = InvertedValue.CounterClockwise_Positive; 
        // Or use Clockwise_Positive if your motor spins the other way

        // Apply those motor output settings
        talonConfig.MotorOutput = motorOutput;

        // 3) Send the config to the TalonFX
        StatusCode configStatus = climberMotor.getConfigurator().apply(talonConfig);
        if (!configStatus.isOK()) {
            System.err.println("Could not configure TalonFX: " + configStatus);
        }
    }

   public void moveUp() {
    double power = SmartDashboard.getNumber("Climber Power", 0.5); 
    climberMotor.setControl(new DutyCycleOut(power));
    } 

public void moveDown() {
    double power = SmartDashboard.getNumber("Climber Power", 0.5);
    climberMotor.setControl(new DutyCycleOut(-power));
    }

  
    public void stop() {
        climberMotor.setControl(new DutyCycleOut(0.0));
    }

    @Override
    public void periodic() {
     
    }
}
