package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.OperatorConstants;
import frc.robot.BooleanReference;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import dev.alphagame.LogManager;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class SwerveDriveCommand extends Command {
    SwerveInputStream driveAngularVelocity;
    BooleanReference crossPressed;

    private final SwerveSubsystem swerveSubsystem;

    PIDController rotationPID;

    public SwerveDriveCommand(SwerveSubsystem swerve, SwerveInputStream input, BooleanReference pressed) {
        rotationPID = new PIDController(0.005, 0.0, 0.0);
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
                double tx = LimelightHelpers.getTX("limelight");
                
                rotationPID.setSetpoint(0);
                final double rotPIDVal = clamp(rotationPID.calculate(tx), -0.7, 0.7);

                // if the targets exist and the distance is accurate but the robot still goes
                // away from the target, invert this.
                boolean pidInvert = false;
                double rotation = (pidInvert ? -1 : 1) * rotPIDVal;
                speeds.omegaRadiansPerSecond = rotation;

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