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
import dev.alphagame.LogManager;

/**
 * Command to align the robot to an AprilTag using Limelight camera feedback.
 * The command will rotate and translate the robot to center on the AprilTag.
 */
public class AlignToAprilTag extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final String limelightName;
    private final double tolerance;
    private static final int CANNOT_FIND_APRIL_TAG_LIMIT = 5; // Number of cycles to wait before stopping
    // PID controller for adjusting rotation to align with target
    private final PIDController rotationPID;
    
    // Constants for controlling alignment speed
    private static final double MAX_ROTATION_SPEED = 1.0; // Maximum rotation speed
    @SuppressWarnings("unused")
    private static final double MAX_TRANSLATION_SPEED = 0.5; // Maximum translation speed
    
    // Add a counter to track how many consecutive loops the robot has been aligned
    private int alignedCounter = 0;
    // Number of consecutive aligned loops required to consider alignment stable
    private static final int ALIGNMENT_STABLE_THRESHOLD = 5;

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
        this.rotationPID = new PIDController(0.2, 0.0, 0.005);
        this.rotationPID.setTolerance(tolerance);
        
        // Add requirements to prevent command conflicts
        addRequirements(swerveSubsystem);
        
        LogManager.info("AlignToAprilTag created with limelight: " + limelightName + ", tolerance: " + tolerance);
    }
    
    /**
     * Simplified constructor that uses the default Limelight name.
     */
    public AlignToAprilTag(SwerveSubsystem swerveSubsystem, double tolerance) {
        this(swerveSubsystem, "", tolerance);
        LogManager.info("AlignToAprilTag created with default limelight, tolerance: " + tolerance);
    }

    @Override
    public void initialize() {
        // Reset PID controller when command starts
        rotationPID.reset();
        LogManager.info("AlignToAprilTag initialized, PID controller reset");
    }

    @Override
    public void execute() {
        // Check if we have a valid target
        boolean hasTarget = LimelightHelpers.getTV(limelightName);
        
        if (!hasTarget) {
            // No valid target, stop the robot
            LogManager.warning("No valid AprilTag target found");
            swerveSubsystem.driveFieldOriented(() -> new ChassisSpeeds(0, 0, 0));
            return;
        }
        
        // Get horizontal offset to target (negative = target is to the left, positive = target is to the right)
        double tx = LimelightHelpers.getTX(limelightName);
        LogManager.debug("Target X offset: " + tx + " degrees");
        
        // Calculate rotation speed using PID controller
        double rotationSpeed = rotationPID.calculate(tx, 0);
        
        // Limit rotation speed
        double limitedRotationSpeed = Math.max(-MAX_ROTATION_SPEED, Math.min(rotationSpeed, MAX_ROTATION_SPEED));
        if (limitedRotationSpeed != rotationSpeed) {
            LogManager.debug("Rotation speed limited from " + rotationSpeed + " to " + limitedRotationSpeed);
        }
        
        // Calculate translation speed - move forward/backward based on target presence
        // This could be enhanced with distance calculation if needed
        double forwardSpeed = 0.0;
        
        // Store final values for use in lambda
        final double finalRotationSpeed = limitedRotationSpeed;
        
        LogManager.debug("Driving with rotation speed: " + finalRotationSpeed);
        
        // Drive the robot using field-oriented control
        swerveSubsystem.driveFieldOriented(() -> new ChassisSpeeds(
            forwardSpeed,
            0.0,  // No sideways movement while aligning
            -finalRotationSpeed  // Negative because positive tx means we need to rotate counterclockwise
        )).execute();

        // Check if we're aligned within tolerance, but don't end the command here
        // We'll track stable alignment in isFinished()
        boolean currentlyAligned = Math.abs(tx) < tolerance;
        if (currentlyAligned) {
            alignedCounter++;
            LogManager.debug("Robot aligned for " + alignedCounter + " consecutive cycles");
        } else {
            // Reset counter if not aligned
            alignedCounter = 0;
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot when command ends
        swerveSubsystem.driveFieldOriented(() -> new ChassisSpeeds(0, 0, 0));
        LogManager.info("AlignToAprilTag command ended, interrupted: " + interrupted);
    }

    @Override
    public boolean isFinished() {
        // Command finishes when we have a valid target and are within tolerance for multiple consecutive cycles

        // Can we see an AprilTag?  If not, exit
        boolean hasTarget = LimelightHelpers.getTV(limelightName);

        if (!hasTarget) {
            LogManager.error("No AprilTag Found!");
            return true;
        }

        // Check if we've been aligned for enough consecutive cycles to consider it stable
        if (alignedCounter >= ALIGNMENT_STABLE_THRESHOLD) {
            double currentTx = LimelightHelpers.getTX(limelightName);
            LogManager.info("AprilTag alignment stable for " + alignedCounter + 
                            " cycles! Final X offset: " + currentTx + " degrees");
            // Do NOT send drive commands here - that's handled in execute() and end()
            return true;
        }
        
        return false;
    }
}

