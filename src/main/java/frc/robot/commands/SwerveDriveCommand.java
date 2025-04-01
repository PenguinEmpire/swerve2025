package frc.robot.commands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.BooleanReference;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class SwerveDriveCommand extends Command {
    SwerveInputStream driveAngularVelocity;
    BooleanReference crossPressed;

    private final SwerveSubsystem swerveSubsystem;

    PIDController rotationPID;

    PIDController strafePID;
  

    public SwerveDriveCommand(SwerveSubsystem swerve, SwerveInputStream input, BooleanReference pressed) {
        // rotationPID = new PIDController(0.02, 0.0, 0.0);
        strafePID = new PIDController(0.02, 0.0, 0.0);

        this.swerveSubsystem = swerve;
        this.driveAngularVelocity = input;
        this.crossPressed = pressed;
        System.out.println("Swerve pressed ref: ");
        System.out.println(this.crossPressed);
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        System.out.println("Running swerve init");
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = driveAngularVelocity.get();

        if (crossPressed.bool) {
            boolean hasTarget = LimelightHelpers.getTV("limelight");

            if (hasTarget) {
                // double tx = LimelightHelpers.getTX("limelight");
                double ty = LimelightHelpers.getTY("limelight");

                
                // rotationPID.setSetpoint(0);
                strafePID.setSetpoint(0);
              
                // final double rotPIDVal = clamp(rotationPID.calculate(tx), -1.0, 1.0);

                final double StrafePIDVal = clamp(strafePID.calculate(ty), -1.0, 1.0);

                // if the targets exist and the distance is accurate but the robot still goes
                // away from the target, invert this.
                boolean pidInvert = false;
                boolean strafeInvert = false;
                // double rotation = (pidInvert ? -1 : 1) * rotPIDVal;
                double strafe = (strafeInvert ? -1 : 1) * StrafePIDVal;
                // speeds.omegaRadiansPerSecond = rotation;
                speeds.vxMetersPerSecond = strafe;

                SmartDashboard.putBoolean("Apriltag Detected?", true);
            }
            else {
                SmartDashboard.putBoolean("Apriltag Detected?", false);
            }
        }
        swerveSubsystem.getSwerveDrive().driveFieldOriented(speeds);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        return;
    }

    public double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}