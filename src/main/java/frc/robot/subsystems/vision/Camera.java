package frc.robot.subsystems.vision;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Camera {

    public final static Camera DRIVE_CAMERA =  new Camera("drive", new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0))); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    //private final static Camera SHOOT_CAMERA =  new Camera("Drive", new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0))); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.

    private final String name;
    private final Transform3d robotToCam;
    public PhotonCamera cam;
    private PhotonPoseEstimator photonPoseEstimator;
    private Pose2d previousPose = new Pose2d();

    public Camera(String name, Transform3d robotToCam) {
              //Forward Camera
      this.name = name;
      this.robotToCam = robotToCam;

      ShuffleboardTab tab = Shuffleboard.getTab("Vision");
      tab.addString("hasCamera", this::hasCamera);
    }
    int i = 0;

    public String hasCamera() {
        return cam.getName() + (getCam() != null) + " - " + cam.isConnected() + " - " + i++;
    }

    public PhotonCamera getCam() {
      if (cam == null) {
        cam = new PhotonCamera(name);
        cam.setPipelineIndex(0);
      }
      return cam;
    }

    public PhotonPoseEstimator getPhotonPoseEstimator() {
      if (photonPoseEstimator == null && getCam() != null) {
        // Construct PhotonPoseEstimator
        photonPoseEstimator = new PhotonPoseEstimator(getFieldLayout(), PoseStrategy.CLOSEST_TO_REFERENCE_POSE, getCam(), robotToCam);
      }
      return photonPoseEstimator;
    }


    public PhotonTrackedTarget getBestTarget() {
        if (getCam() != null) {
            PhotonPipelineResult result = getCam().getLatestResult();
            if (result.hasTargets())
                return result.getBestTarget();
        }
        return null;
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        if (getPhotonPoseEstimator() != null) {
            photonPoseEstimator.setReferencePose(previousPose);
            Optional<EstimatedRobotPose> pose = photonPoseEstimator.update();
            if (pose.isPresent()) {
                previousPose = pose.get().estimatedPose.toPose2d();
            }
            return pose;
        }
        return null;
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
            layout.setOrigin(alliance == Alliance.Blue ?  OriginPosition.kBlueAllianceWallRightSide : OriginPosition.kRedAllianceWallRightSide);
        } catch(IOException e) {
            DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
            layout = null;
        }
        return layout;
    }
    
}
