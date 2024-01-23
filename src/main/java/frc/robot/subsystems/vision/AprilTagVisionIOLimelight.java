
package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.TimestampedString;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.ArrayList;
import java.util.Optional;

public class AprilTagVisionIOLimelight {

  String limelightName;
  private final StringSubscriber[] observationSubscribers;

  public AprilTagVisionIOLimelight(String... identifiers) {
    ArrayList<StringSubscriber> observers = new ArrayList<StringSubscriber>();

    for (String id :identifiers) {
      NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable(id);
      LimelightHelpers.setPipelineIndex(limelightName, 0);

      observers.add(
          limelightTable
              .getStringTopic("json")
              .subscribe("", PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true))
      );
    }
    observationSubscribers = observers.toArray(new StringSubscriber[0]);
  }

  public ArrayList<PoseEstimate> getVisionData() {
    ArrayList<PoseEstimate> poseEstimates = new ArrayList<>();
    for (StringSubscriber observationSubscriber: observationSubscribers) {
      TimestampedString[] queue = observationSubscriber.readQueue();
      for (TimestampedString timestampedString : queue) {
        double timestamp = timestampedString.timestamp / 1e6;
        LimelightHelpers.Results results =
            LimelightHelpers.parseJsonDump(timestampedString.value).targetingResults;
        Optional<Alliance> allianceOptional = DriverStation.getAlliance();
        if (results.targets_Fiducials.length == 0 || !allianceOptional.isPresent()) {
          continue;
        }
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
    }
    return poseEstimates;
  }
}