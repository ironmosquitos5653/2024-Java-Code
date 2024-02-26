package frc.robot.subsystems.vision;

import java.io.IOException;
import java.util.ArrayList;
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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

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

    private Pose2d previousPose;

    private final PhotonPoseEstimator photonPoseEstimator;
    private PhotonCamera camera;

    public Camera(String name, Transform3d robotToCam) {
        this.name = name;
        this.robotToCam = robotToCam;
        camera = new PhotonCamera(name);

        photonPoseEstimator = new PhotonPoseEstimator(getFieldLayout(), PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, robotToCam);
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        photonPoseEstimator.setReferencePose(previousPose);
        Optional<EstimatedRobotPose> estimatedPose = photonPoseEstimator.update();
        if(estimatedPose.isPresent()) {
            previousPose = estimatedPose.get().estimatedPose.toPose2d();
        }
        return estimatedPose;
    }

    public PhotonCamera getPhotonCamera() {
        return camera;
    }

    public Transform3d getRobotToCamera() {
        return robotToCam;
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

}