package frc.robot.subsystems.vision;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.TimestampedString;
import edu.wpi.first.networktables.Topic;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.vision.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.subsystems.vision.LimelightHelpers.Results;

public class Camera {

    public final static Camera DRIVE_CAMERA =  new Camera("limelight-drive", new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0))); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    public final static Camera SHOOT_CAMERA =  new Camera("limelight-shoot", new Transform3d(new Translation3d(0.23495, 0.3175, 0.0635), new Rotation3d(0,0,180))); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
// 9.25
// 12.5
// 2.5
    private final String name;
    private final Transform3d robotToCam;
    private Pose2d previousPose = new Pose2d();

    private final StringSubscriber observationSubscriber;

    public Camera(String name, Transform3d robotToCam) {
              //Forward Camera
      this.name = name;
      this.robotToCam = robotToCam;

      ShuffleboardTab tab = Shuffleboard.getTab("Vision");

      NetworkTable limelightTable =  LimelightHelpers.getLimelightNTTable(name);

      observationSubscriber =
        limelightTable
            .getStringTopic("json")
            .subscribe("", PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));

    }

    public String getTopics() {
      NetworkTable limelightTable =  LimelightHelpers.getLimelightNTTable(name);
        StringBuilder sb = new StringBuilder();
        for(Topic topic : limelightTable.getTopics()) {
            sb.append(topic.getName() + "   ");
        }

        return sb.toString();
    }

    /* 
    public double distanceToTarget() {
        PhotonPipelineResult result = cam.getLatestResult();
        if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            double range =
            PhotonUtils.calculateDistanceToTargetMeters(
                38, //CAMERA_HEIGHT_METERS,
                getFieldLayout().getTagPose(target.getFiducialId()).get().getZ()
                CAMERA_PITCH_RADIANS,
                Units.degreesToRadians(().getPitch()));
        }

        return -1;
    }
    */

    private static AprilTagFieldLayout layout;

    public static AprilTagFieldLayout getFieldLayout() {
        try {
            layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
            var alliance = DriverStation.getAlliance().get();
            //layout.setOrigin(alliance == Alliance.Blue ?  OriginPosition.kBlueAllianceWallRightSide : OriginPosition.kRedAllianceWallRightSide);
            layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);



        } catch(IOException e) {
            DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
            layout = null;
        }
        return layout;
    }

    private Pose3d getPose(Results results) {
        var alliance = DriverStation.getAlliance();
        if(alliance.isEmpty() || alliance.get() == Alliance.Blue)
        {
            return results.getBotPose3d_wpiBlue();
        }
        return results.getBotPose3d_wpiRed();
    }

    public ArrayList<PoseInstance> getLatest() {
        TimestampedString[] queue = observationSubscriber.readQueue();
        ArrayList<PoseInstance> poseEstimates = new ArrayList<>();
        for (TimestampedString timestampedString : queue) {
            var results = LimelightHelpers.getLatestResults(name).targetingResults;
            SmartDashboard.putString("results", results.toString());
            double timestamp = (timestampedString.timestamp - results.latency_capture - results.latency_pipeline) / 1e6;

            if (results.targets_Fiducials.length > 0) {

                Pose3d pose = getPose(results);
                
                // TODO:  Calculate target angle.

                double distance = getAvgTA(results.targets_Fiducials);

                poseEstimates.add(new PoseInstance(pose.toPose2d(), timestamp));
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
}