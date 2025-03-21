package frc.robot.subsystems;

import java.util.EnumSet;
import java.util.Optional;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.MultiSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableListenerPoller;
import edu.wpi.first.networktables.NetworkTableValue;

public class PiVision {
    final MultiSubscriber multisub;
    final NetworkTableListenerPoller poller;

    public PiVision() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        multisub = new MultiSubscriber(inst, new String[] {"/Cam/NW", "/Cam/NE", "/Cam/SE", "/Cam/SW"});
        poller = new NetworkTableListenerPoller(inst);
        poller.addListener(multisub, EnumSet.of(NetworkTableEvent.Kind.kValueAll));
    }
    
    public void updateCameraOdometry() {
        NetworkTableEvent[] events = poller.readQueue();
        for (NetworkTableEvent event : events) {
            double[] camPos = event.valueData.value.getDoubleArray();
            double currTs = Timer.getFPGATimestamp();
            System.out.printf("%s={x: %f, y: %f, latency: %d, timestamp: %d}", event.topicInfo.name, camPos[0], camPos[1], currTs, event.timeSyncData.serverTimeOffset);
        }
        /*
        for (Cameras camera : Cameras.values())
            {
            Optional<EstimatedRobotPose> poseEst = getEstimatedGlobalPose(camera);
            if (poseEst.isPresent())
            {
             var pose = poseEst.get();
        }
             */
    }

    public void close() {
        poller.close();
        multisub.close();
    }
}

    /*
    swerveDrive.addVisionMeasurement(pose.estimatedPose.toPose2d(),
                                        pose.timestampSeconds,
                                        camera.curStdDevs);
     */
