package org.penguinempire.commands;
import org.penguinempire.BooleanReference;
import org.penguinempire.LimelightHelpers;
import org.penguinempire.subsystems.SwerveSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import swervelib.SwerveInputStream;

public class SwerveDriveCommand extends Command {
    SwerveInputStream driveAngularVelocity;
    BooleanReference L3Pressed;
    BooleanReference PsPressed;
    BooleanReference R3Pressed;

    private final SwerveSubsystem swerveSubsystem;

    PIDController rotationPID;

    PIDController strafePID;
  
    boolean hold = false;

    public SwerveDriveCommand(SwerveSubsystem swerve, SwerveInputStream input, BooleanReference L3Pressed, BooleanReference PsPressed, BooleanReference R3Pressed) {
        rotationPID = new PIDController(0.02, 0.0, 0.0);
        strafePID = new PIDController(0.25, 0.0, 0.0);
        strafePID.setTolerance(1);

        this.swerveSubsystem = swerve;
        this.driveAngularVelocity = input;
        this.L3Pressed = L3Pressed;
        this.PsPressed = PsPressed;
        this.R3Pressed = R3Pressed;
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

        if (L3Pressed.bool || PsPressed.bool || R3Pressed.bool) {
            boolean hasTarget = LimelightHelpers.getTV("limelight");

            if (hasTarget) {
                @SuppressWarnings("unused")
                double offset = SmartDashboard.getNumber("Offset", 0);
                double[] positions = LimelightHelpers.getCameraPose_TargetSpace("limelight");
                System.out.println(SmartDashboard.getBoolean("LOffset", false));
                //double tx = LimelightHelpers.getTX("limelight");
                //double ty = LimelightHelpers.getTY("limelight");

                //rotationPID.setSetpoint(0);
                strafePID.setSetpoint(0);
              
                double xOffset = positions[0];

                if (L3Pressed.bool) {
                    offset -= 0.25;
                } else if (R3Pressed.bool) {
                    offset += 0.25;
                }

                //final double rotPIDVal = clamp(rotationPID.calculate(tx), -1.0, 1.0);
                final double StrafePIDVal = strafePID.calculate(xOffset);

                // if the targets exist and the distance is accurate but the robot still goes
                // away from the target, invert this.
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