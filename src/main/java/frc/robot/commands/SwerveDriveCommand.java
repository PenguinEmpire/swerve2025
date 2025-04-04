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
  
    boolean hold = false;

    public SwerveDriveCommand(SwerveSubsystem swerve, SwerveInputStream input, BooleanReference pressed) {
        rotationPID = new PIDController(0.02, 0.0, 0.0);
        strafePID = new PIDController(0.25, 0.0, 0.0);
        strafePID.setTolerance(1);

        this.swerveSubsystem = swerve;
        this.driveAngularVelocity = input;
        this.crossPressed = pressed;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        System.out.println("Running swerve init");
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = driveAngularVelocity.get();

        if (LimelightHelpers.getTV("limelight")) {
            SmartDashboard.putNumber("Dash TY", LimelightHelpers.getTY("limelight"));
        }

        if (crossPressed.bool) {
            boolean hasTarget = LimelightHelpers.getTV("limelight");

            if (hasTarget) {
                double offset = SmartDashboard.getNumber("Offset", 0);
                double[] positions = LimelightHelpers.getCameraPose_TargetSpace("limelight");
                System.out.println(SmartDashboard.getBoolean("LOffset", false));
                //double tx = LimelightHelpers.getTX("limelight");
                //double ty = LimelightHelpers.getTY("limelight");

                //rotationPID.setSetpoint(0);
                strafePID.setSetpoint(0);
              
                double xOffset = positions[0];

                if (SmartDashboard.getBoolean("LOffset", false)) {
                    offset -= 0.25;
                } else if (SmartDashboard.getBoolean("ROffset", false)) {
                    offset += 0.25;
                }

                //final double rotPIDVal = clamp(rotationPID.calculate(tx), -1.0, 1.0);
                final double StrafePIDVal = strafePID.calculate(xOffset);

                // if the targets exist and the distance is accurate but the robot still goes
                // away from the target, invert this.
                boolean pidInvert = false;
                boolean strafeInvert = true;
                //double rotation = (pidInvert ? -1 : 1) * rotPIDVal;
                double strafe = (strafeInvert ? -1 : 1) * StrafePIDVal;
                //speeds.omegaRadiansPerSecond = rotation;
                speeds.vxMetersPerSecond = strafe;

                SmartDashboard.putBoolean("Apriltag Detected?", true);
            }
            else {
                if (hold) {
                    speeds.vxMetersPerSecond = 0;
                }
                SmartDashboard.putBoolean("Apriltag Detected?", false);
            }
        }
        else {
            hold = false;
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