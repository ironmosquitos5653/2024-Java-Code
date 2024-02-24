// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.Camera;
import frc.robot.subsystems.vision.PoseEstimator.TimestampedVisionUpdate;
import frc.robot.subsystems.vision.VisionHelpers.PoseEstimate;

public class PhotonVisionPoseEstimatorSubsystem extends SubsystemBase {

  private final AprilTagFieldLayout kTagLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
  private PhotonPoseEstimator photonEstimator;
  private Camera camera;

  public PhotonVisionPoseEstimatorSubsystem() {
  }

  @Override
  public void periodic() {
      ArrayList<PoseEstimate> estimates = Camera.SHOOT_CAMERA.getEstimates();
    List<TimestampedVisionUpdate> visionUpdates = processPoseEstimates();
    //sendResultsToPoseEstimator(visionUpdates);
  }

    private List<TimestampedVisionUpdate> processPoseEstimates() {
    List<TimestampedVisionUpdate> visionUpdates = new ArrayList<>();
    /*
    for (int instanceIndex = 0; instanceIndex < io.length; instanceIndex++) {
      for (PoseEstimate poseEstimates : inputs[instanceIndex].poseEstimates) {
        if (shouldSkipPoseEstimate(poseEstimates)) {
          continue;
        }
        double timestamp = poseEstimates.timestampSeconds();
        Pose3d robotPose3d = poseEstimates.pose();
        setRobotPose(robotPose3d.toPose2d());
        List<Pose3d> tagPoses = getTagPoses(poseEstimates);
        double xyStdDev = calculateXYStdDev(poseEstimates, tagPoses.size());
        double thetaStdDev = calculateThetaStdDev(poseEstimates, tagPoses.size());
        visionUpdates.add(
            new TimestampedVisionUpdate(
                timestamp, getRobotPose(), VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)));
        logData(instanceIndex, timestamp, robotPose3d, tagPoses);
      }
    }
    */
    return visionUpdates;
  }
}
