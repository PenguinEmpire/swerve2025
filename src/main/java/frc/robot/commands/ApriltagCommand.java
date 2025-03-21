package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class ApriltagCommand extends Command {
    private final SwerveSubsystem swerve;
    private final PIDController strafeController = new PIDController(0.05, 0, 0);
    private final PIDController forwardController = new PIDController(0.05, 0, 0);
    private final PIDController rotationController = new PIDController(0.02, 0, 0);

    private static final double TARGET_DISTANCE = 0.2;
    private static final double POSITION_TOLERANCE = 0.05;
    private static final double ROTATION_TOLERANCE = 2.0;

    private Pose2d targetPose;

    public ApriltagCommand(SwerveSubsystem swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("limelight");

        if (table.getEntry("tv").getDouble(0) == 0) {
            System.out.println("No apriltag detected");
            cancel();
            return;
        }

        double tx = table.getEntry("tx").getDouble(0) / 100 - TARGET_DISTANCE;
        double ty = table.getEntry("ty").getDouble(0) / 100 - TARGET_DISTANCE;
        double[] botPose = table.getEntry("botpose").getDoubleArray(new double[6]);
        double tr = botPose[5];

        Pose2d currentPose = swerve.getPose();
        targetPose = new Pose2d(
            new Translation2d(currentPose.getX() + ty, currentPose.getY() + tx),
            Rotation2d.fromDegrees(tr)
        );
    }

    @Override
    public void execute() {
        Pose2d currentPose = swerve.getPose();
        double strafeSpeed = strafeController.calculate(currentPose.getY(), targetPose.getY());
        double forwardSpeed = forwardController.calculate(currentPose.getX(), targetPose.getX());
        double rotationSpeed = rotationController.calculate(currentPose.getRotation().getDegrees(), targetPose.getRotation().getDegrees());

        // Apply deadband for smoother stopping
        if (Math.abs(currentPose.getX() - targetPose.getX()) < POSITION_TOLERANCE) forwardSpeed = 0;
        if (Math.abs(currentPose.getY() - targetPose.getY()) < POSITION_TOLERANCE) strafeSpeed = 0;
        if (Math.abs(currentPose.getRotation().getDegrees() - targetPose.getRotation().getDegrees()) < ROTATION_TOLERANCE) rotationSpeed = 0;

        swerve.drive(forwardSpeed, strafeSpeed, rotationSpeed);
    }

    @Override
    public boolean isFinished() {
        Pose2d currentPose = swerve.getPose();

        boolean positionReached = currentPose.getTranslation().getDistance(targetPose.getTranslation()) < POSITION_TOLERANCE;
        boolean rotationReached = Math.abs(currentPose.getRotation().getDegrees() - targetPose.getRotation().getDegrees()) < ROTATION_TOLERANCE;

        return positionReached && rotationReached;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(0, 0, 0);
    }
}
