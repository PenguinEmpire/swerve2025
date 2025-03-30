// Copyright (c) 2025 Damien Boisvert (AlphaGameDeveloper)
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

package frc.robot.commands;

import dev.alphagame.LogManager;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
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
    private static final int CANNOT_FIND_APRIL_TAG_LIMIT = 5; // Number of cycles to wait before stopping
    
    // PID controllers for alignment
    private final PIDController rotationPID;
    private final PIDController lateralPID;
    
    // Counter to track stable alignment
    private int alignedCounter = 0;
    private static final int ALIGNMENT_STABLE_THRESHOLD = 5;
    
    // Constants for controlling alignment speed
    private static final double MAX_ROTATION_SPEED = 1.0; // Maximum rotation speed
    private static final double MAX_LATERAL_SPEED = 0.5; // Maximum lateral movement speed

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
        
        // Configure PID controllers for alignment
        this.rotationPID = new PIDController(0.010, 0.0, 0.00);

        this.rotationPID.setTolerance(tolerance);
        
        // Add lateral movement PID controller
        this.lateralPID = new PIDController(0.05, 0.0, 0.0);
        this.lateralPID.setTolerance(tolerance);


        
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
        // Reset PID controllers when command starts
        rotationPID.reset();
        lateralPID.reset();
        alignedCounter = 0;
        LogManager.info("AlignToAprilTag initialized, PID controllers reset");
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
        
        // Get traditional TX value first as fallback
        double tx = LimelightHelpers.getTX(limelightName);
        
        // Get 3D pose data with error handling
        double x = 0;
        double z = 0;
        double yawToTarget = 0;
        boolean using3DPose = false;
        
        try {
            // Try to get 3D pose data
            Pose3d targetPose = LimelightHelpers.getTargetPose3d_CameraSpace(limelightName);
            
            // Verify we got valid pose data
            if (targetPose != null && !Double.isNaN(targetPose.getX()) && !Double.isNaN(targetPose.getZ())) {
                // Extract position components from the target pose
                x = targetPose.getX(); // Left/right offset (+ is right from camera's view)
                z = targetPose.getZ(); // Depth/distance to target
                
                // Only use 3D data if the values are reasonably sized
                if (Math.abs(x) < 100 && Math.abs(z) < 100 && z != 0) {
                    double distance = Math.sqrt(x*x + z*z); // Direct distance to target
                    yawToTarget = Math.toDegrees(Math.atan2(x, z)); // Angle from camera to target
                    using3DPose = true;
                    
                    LogManager.debug(String.format("Using 3D pose - x=%.2f, z=%.2f, dist=%.2f, yaw=%.2f°", 
                                                 x, z, distance, yawToTarget));
                }
            }
        } catch (Exception e) {
            LogManager.error("Error getting 3D pose: " + e.getMessage());
        }
        
        // Fall back to TX values if 3D pose isn't usable
        if (!using3DPose) {
            LogManager.warning("3D pose data invalid, falling back to 2D tracking");
            yawToTarget = tx; // Use TX as the yaw angle
            x = Math.sin(Math.toRadians(tx)) * 2.0; // Approximate lateral offset based on angle
        }
        
        // Debug output
        LogManager.debug(String.format("Alignment data - tx=%.2f°, 3D yaw=%.2f°, lateral offset=%.2f", 
                                      tx, yawToTarget, x));
        
        // Gradient ascent logic for lateral and rotational alignment
        double rotationSpeed = rotationPID.calculate(yawToTarget, 0);
        double lateralSpeed = lateralPID.calculate(x, 0);

        // Apply gradient ascent scaling
        rotationSpeed *= Math.abs(yawToTarget);
        lateralSpeed *= Math.abs(x);

        // Limit rotation and lateral speeds
        double limitedRotationSpeed = Math.max(- MAX_ROTATION_SPEED, Math.min(rotationSpeed, MAX_ROTATION_SPEED));
        double limitedLateralSpeed = Math.max(-MAX_LATERAL_SPEED, Math.min(lateralSpeed, MAX_LATERAL_SPEED));
        
        // Forward speed remains 0 - we're only focusing on lateral and rotational alignment
        double forwardSpeed = 0.0;
        
        // Store final values for use in lambda
        final double finalRotationSpeed = limitedRotationSpeed;
        final double finalLateralSpeed = -limitedLateralSpeed; // Note sign - depends on coordinate system
        
        LogManager.info("Driving with rotation speed: " + finalRotationSpeed + ", lateral speed: " + finalLateralSpeed);
        
        // Drive the robot using field-oriented control
        swerveSubsystem.driveFieldOriented(() -> new ChassisSpeeds(
            forwardSpeed, 
            finalLateralSpeed,
            finalRotationSpeed  // Negative because positive yaw means we need to rotate counterclockwise
        )).execute();

        // Check alignment based on distance to the ideal position (directly in front of tag)
        boolean rotationallyAligned = Math.abs(yawToTarget) < tolerance;
        boolean laterallyAligned = Math.abs(x) < (using3DPose ? tolerance * 0.05 : tolerance);
        boolean currentlyAligned = rotationallyAligned && laterallyAligned;
        
        if (currentlyAligned) {
            alignedCounter++;
            LogManager.debug("Robot aligned for " + alignedCounter + " consecutive cycles" +
                            " (yaw: " + yawToTarget + "°, lateral: " + x + ")");
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
        // Check if we have a valid target
        boolean hasTarget = LimelightHelpers.getTV(limelightName);

        if (!hasTarget) {
            LogManager.error("No AprilTag Found!");
            return true;
        }

        // Check if we've been aligned for enough consecutive cycles
        if (alignedCounter >= ALIGNMENT_STABLE_THRESHOLD) {
            LogManager.info("AprilTag alignment complete! " + alignedCounter + " stable cycles.");
            return true;
        }
        
        // Not yet stable for enough cycles
        return false;
    }
}

