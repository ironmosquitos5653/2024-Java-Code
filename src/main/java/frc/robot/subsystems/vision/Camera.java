package frc.robot.subsystems.vision;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Camera {

    public final static Camera DRIVE_CAMERA =  new Camera("Drive", new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0))); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    //private final static Camera SHOOT_CAMERA =  new Camera("Drive", new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0))); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.

    private final PhotonPoseEstimator photonPoseEstimator;
    private Pose2d previousPose = null;

    public Camera(String name, Transform3d robotToCam) {
              //Forward Camera
      PhotonCamera cam = new PhotonCamera("testCamera");

      // Construct PhotonPoseEstimator
      photonPoseEstimator = new PhotonPoseEstimator(getFieldLayout(), PoseStrategy.CLOSEST_TO_REFERENCE_POSE, cam, robotToCam);
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        photonPoseEstimator.setReferencePose(previousPose);
        Optional<EstimatedRobotPose> pose = photonPoseEstimator.update();
        if (pose.isPresent()) {
            previousPose = pose.get().estimatedPose.toPose2d();
        }
        return pose;
    }

    private static AprilTagFieldLayout layout;

    public static AprilTagFieldLayout getFieldLayout() {
        try {
            layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
            var alliance = DriverStation.getAlliance().get();
            layout.setOrigin(alliance == Alliance.Blue ?  OriginPosition.kBlueAllianceWallRightSide : OriginPosition.kRedAllianceWallRightSide);
        } catch(IOException e) {
            DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
            layout = null;
        }
        return layout;
    }
    
}
