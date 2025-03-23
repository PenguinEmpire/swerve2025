// Copyright (c) 2025 Damien Boisvert (AlphaGameDeveloper)
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * Command to align the robot to an AprilTag using Limelight camera feedback.
 * The command will rotate and translate the robot to center on the AprilTag.
 */
public class AlignToAprilTag extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final String limelightName;
    private final double tolerance;
    
    // PID controller for adjusting rotation to align with target
    private final PIDController rotationPID;
    
    // Constants for controlling alignment speed
    private static final double MAX_ROTATION_SPEED = 1.0; // Maximum rotation speed
    private static final double MAX_TRANSLATION_SPEED = 0.5; // Maximum translation speed

    /**
     * Creates a new AlignToAprilTag command.
     * 
     * @param swerveSubsystem The swerve subsystem used for movement
     * @param limelightName The name of the Limelight camera ("" for default)
     * @param tolerance The acceptable error margin in degrees
     */
    public AlignToAprilTag(SwerveSubsystem swerveSubsystem, String limelightName, double tolerance) {
        this.swerveSubsystem = swerveSubsystem;
        this.limelightName = limelightName;
        this.tolerance = tolerance;
        
        // Configure PID controller for alignment
        this.rotationPID = new PIDController(0.03, 0.0, 0.005);
        this.rotationPID.setTolerance(tolerance);
        
        // Add requirements to prevent command conflicts
        addRequirements(swerveSubsystem);
    }
    
    /**
     * Simplified constructor that uses the default Limelight name.
     */
    public AlignToAprilTag(SwerveSubsystem swerveSubsystem, double tolerance) {
        this(swerveSubsystem, "", tolerance);
    }

    @Override
    public void initialize() {
        // Reset PID controller when command starts
        rotationPID.reset();
    }

    @Override
    public void execute() {
        // Check if we have a valid target
        if (!LimelightHelpers.getTV(limelightName)) {
            // No valid target, stop the robot
            swerveSubsystem.driveFieldOriented(() -> new ChassisSpeeds(0, 0, 0));
            return;
        }
        
        // Get horizontal offset to target (negative = target is to the left, positive = target is to the right)
        double tx = LimelightHelpers.getTX(limelightName);
        
        // Calculate rotation speed using PID controller
        double rotationSpeed = rotationPID.calculate(tx, 0);
        
        // Limit rotation speed
        rotationSpeed = Math.max(-MAX_ROTATION_SPEED, Math.min(rotationSpeed, MAX_ROTATION_SPEED));
        
        // Calculate translation speed - move forward/backward based on target presence
        // This could be enhanced with distance calculation if needed
        double forwardSpeed = 0.0;
        
        // Store final values for use in lambda
        final double finalRotationSpeed = rotationSpeed;
        
        // Drive the robot using field-oriented control
        swerveSubsystem.driveFieldOriented(() -> new ChassisSpeeds(
            forwardSpeed,
            0.0,  // No sideways movement while aligning
            -finalRotationSpeed  // Negative because positive tx means we need to rotate counterclockwise
        ));
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot when command ends
        swerveSubsystem.driveFieldOriented(() -> new ChassisSpeeds(0, 0, 0));
    }

    @Override
    public boolean isFinished() {
        // Command finishes when we have a valid target and are within tolerance
        return LimelightHelpers.getTV(limelightName) && 
               Math.abs(LimelightHelpers.getTX(limelightName)) < tolerance;
    }
}

