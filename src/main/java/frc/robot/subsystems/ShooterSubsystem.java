package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Shooter;

public class ShooterSubsystem extends SubsystemBase {
    private final SparkMax shooterMotor;
    private final RelativeEncoder limitSwitchEncoder;  //  Treat limit switch as encoder input

    private static final double LIMIT_SWITCH_THRESHOLD = 0.1;  //  Adjust based on real values

    public ShooterSubsystem() {
        shooterMotor = new SparkMax(Shooter.SHOOTER_MOTOR_ID, MotorType.kBrushless);

        //  Configure motor settings
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(false); // Adjust based on shooter direction
        shooterMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //  Get encoder from the Alternate Encoder Port (Limit switch input)
        limitSwitchEncoder = shooterMotor.getAlternateEncoder(); // Adjust CPR if needed
    }

    /** Reads the limit switch state based on encoder behavior */
    public boolean isLimitSwitchPressed() {
        double encoderPosition = limitSwitchEncoder.getPosition(); //  Read encoder position

        //  If encoder position is below the threshold, assume limit switch is pressed
        return Shooter.ALT_ENC_LIMIT && encoderPosition < LIMIT_SWITCH_THRESHOLD;
    }

    @Override
    public void periodic() {
        //  Get limit switch state
        boolean limitSwitchActive = isLimitSwitchPressed();

        //  Display limit switch state on SmartDashboard
        SmartDashboard.putBoolean("Shooter Limit Switch", limitSwitchActive);
        SmartDashboard.putNumber("Limit Switch Encoder Position", limitSwitchEncoder.getPosition()); // ✅Debugging position
    }
}
