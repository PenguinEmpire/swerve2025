package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Climber;

public class ClimberSubsystem extends SubsystemBase {
    private final SparkMax ClimberMotor;
    // private final RelativeEncoder limitSwitchEncoder;  // Treat limit switch as encoder input

    // private static final double LIMIT_SWITCH_THRESHOLD = 0.1;  // Adjust based on real values

    private double ClimberPower = Climber.DEFAULT_CLIMBER_POWER; 
   

    public ClimberSubsystem() {
        ClimberMotor = new SparkMax(Climber.CLIMBER_MOTOR_ID, MotorType.kBrushless);

        // Configure motor settings
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(false); // Adjust based on shooter direction
        ClimberMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Get encoder from the Alternate Encoder Port (Limit switch input)
        // limitSwitchEncoder = shooterMotor.getAlternateEncoder(); // Adjust CPR if needed

        // Initialize SmartDashboard with power settings
        SmartDashboard.putNumber("Shooter Power", Climber.DEFAULT_CLIMBER_POWER);
    }

    // /** Reads the limit switch state based on encoder behavior */
    // public boolean isLimitSwitchPressed() {
    //     double encoderPosition = limitSwitchEncoder.getPosition(); // Read encoder position

    //     // If encoder position is below the threshold, assume limit switch is pressed
    //     return Shooter.ALT_ENC_LIMIT && encoderPosition < LIMIT_SWITCH_THRESHOLD;
    // }

    /** Spins the shooter motor in forward or reverse direction */
    public void spinShooter(boolean forward) {
        ClimberPower = SmartDashboard.getNumber("Shooter Power", ClimberPower);  // Get shooter power from SmartDashboard

        double power = forward ? ClimberPower : -ClimberPower;  // Adjust direction based on forward/reverse
         ClimberMotor.set(power);  // Set motor power
    }

    /** Stops the shooter motor */
    public void stopShooter() {
        ClimberMotor.set(0);  // Stop the shooter motor
    }

    @Override
    public void periodic() {
        // Get limit switch state
        // boolean limitSwitchActive = isLimitSwitchPressed();

        // Display limit switch state on SmartDashboard
        // SmartDashboard.putBoolean("Shooter Limit Switch", limitSwitchActive);
        // SmartDashboard.putNumber("Limit Switch Encoder Position", limitSwitchEncoder.getPosition()); // Debugging position

        // Optionally display the current shooter power for debugging
        SmartDashboard.putNumber("Shooter Power",ClimberPower);  // Display percentage of power
    }
}
