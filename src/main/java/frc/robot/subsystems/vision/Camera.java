package frc.robot.subsystems.vision;

import java.io.IOException;
import java.util.ArrayList;

import org.photonvision.PhotonCamera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.TimestampedString;
import edu.wpi.first.networktables.Topic;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.vision.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.subsystems.vision.LimelightHelpers.Results;
import frc.robot.subsystems.vision.VisionHelpers.PoseEstimate;

public class Camera {

    /*
    public final static Camera DRIVE_CAMERA =  new Camera("limelight-drive",
            new Transform2d(new Translation2d(0.2286, .2032),
                    new Rotation2d(0)));
    */

    public final static Camera SHOOT_CAMERA =  new Camera("limelight-shoot",
            new Transform3d(new Translation3d(0.23495, 0.3175, 0),
                    new Rotation3d(Units.degreesToRadians(180),0,0)));

    private final String name;
    private final Transform3d robotToCam;

    private final StringSubscriber observationSubscriber;
    private PhotonCamera camera;

    public Camera(String name, Transform3d robotToCam) {
              //Forward Camera
      this.name = name;
      this.robotToCam = robotToCam;
      camera = new PhotonCamera(name);

      NetworkTable limelightTable =  LimelightHelpers.getLimelightNTTable(name);

      observationSubscriber =
        limelightTable
            .getStringTopic("json")
            .subscribe("", PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));

    }

    public PhotonCamera getPhotonCamera() {
        return camera;
    }

    public Transform3d getRobotToCamera() {
        return robotToCam;
    }

    public String getTopics() {
      NetworkTable limelightTable =  LimelightHelpers.getLimelightNTTable(name);
        StringBuilder sb = new StringBuilder();
        for(Topic topic : limelightTable.getTopics()) {
            sb.append(topic.getName() + "   ");
        }

        return sb.toString();
    }

    private static AprilTagFieldLayout layout;

    public static AprilTagFieldLayout getFieldLayout() {
        try {
            layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
            //layout.setOrigin(alliance == Alliance.Blue ?  OriginPosition.kBlueAllianceWallRightSide : OriginPosition.kRedAllianceWallRightSide);
            layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

        } catch(IOException e) {
            DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
            layout = null;
        }
        return layout;
    }

    private Pose2d getPose(Results results) {
        var alliance = DriverStation.getAlliance();
        if(alliance.isEmpty() || alliance.get() == Alliance.Blue)
        {
            return results.getBotPose2d_wpiBlue();
        }
        return results.getBotPose2d_wpiRed();
    }

    public ArrayList<PoseInstance> getLatest() {
        TimestampedString[] queue = observationSubscriber.readQueue();
        ArrayList<PoseInstance> poseEstimates = new ArrayList<>();
        for (TimestampedString timestampedString : queue) {
            var results = LimelightHelpers.getLatestResults(name).targetingResults;
            double timestamp = (timestampedString.timestamp - results.latency_capture - results.latency_pipeline) / 1e6;

            if (results.targets_Fiducials.length > 0) {

                Pose2d pose = getPose(results);
                Pose2d transformed =
                    new Pose2d(
                       pose.getX() + getFieldLayout().getFieldLength()/2,
                       pose.getY() + getFieldLayout().getFieldWidth()/2,
                       new Rotation2d(pose.getRotation().getRadians()))
                    ;//.transformBy(robotToCam.inverse());

                poseEstimates.add(new PoseInstance(pose, timestamp));
            }
        }
        return poseEstimates;
    }

    public double getAvgTA(LimelightTarget_Fiducial[] fiducials){
        double sumTA = 0;
        for(int i = 0; i < fiducials.length; i++){
        sumTA += fiducials[i].ta;
        }
        return sumTA / fiducials.length;
    }

    public class PoseInstance {
        private Pose2d pose;
        public Pose2d getPose() { return pose;}

        private double timestamp;
        public double getTimestamp() { return timestamp;}
        
        public PoseInstance(Pose2d pose, double timestamp) {
            this.pose = pose;
            this.timestamp = timestamp;
        }
    }

    public ArrayList<PoseEstimate> getEstimates() {
        TimestampedString[] queue = observationSubscriber.readQueue();
        ArrayList<PoseEstimate> poseEstimates = new ArrayList<PoseEstimate>();
        for (TimestampedString timestampedString : queue) {
            double timestamp = timestampedString.timestamp / 1e6;
            LimelightHelpers.Results results =
                LimelightHelpers.parseJsonDump(timestampedString.value).targetingResults;
                
            double latencyMS = results.latency_capture + results.latency_pipeline;
            Pose3d poseEstimation = results.getBotPose3d_wpiBlue();
            double averageTagDistance = 0.0;
            timestamp -= (latencyMS / 1e3);
            int[] tagIDs = new int[results.targets_Fiducials.length];
            for (int i = 0; i < results.targets_Fiducials.length; i++) {
                tagIDs[i] = (int) results.targets_Fiducials[i].fiducialID;
                averageTagDistance +=
                    results.targets_Fiducials[i].getTargetPose_CameraSpace().getTranslation().getNorm();
            }
            averageTagDistance /= tagIDs.length;
            poseEstimates.add(new PoseEstimate(poseEstimation, timestamp, averageTagDistance, tagIDs));
        }
        return poseEstimates;
    }
}