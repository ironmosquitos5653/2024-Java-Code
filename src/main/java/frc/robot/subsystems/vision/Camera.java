package frc.robot.subsystems.vision;

import java.io.IOException;
import java.util.ArrayList;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
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

public class Camera {

    public final static Camera DRIVE_CAMERA =  new Camera("limelight-drive",
            new Transform2d(new Translation2d(0.2286, .2032),
                    new Rotation2d(0)));

    public final static Camera SHOOT_CAMERA =  new Camera("limelight-shoot",
            new Transform2d(new Translation2d(0.23495, 0.3175),
                    new Rotation2d(Units.degreesToRadians(180))));

    private final String name;
    private final Transform2d robotToCam;

    private final StringSubscriber observationSubscriber;

    public Camera(String name, Transform2d robotToCam) {
              //Forward Camera
      this.name = name;
      this.robotToCam = robotToCam;

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
}