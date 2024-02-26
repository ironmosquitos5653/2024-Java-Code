package frc.robot.subsystems;


import java.util.ArrayList;
import java.util.List;
import java.util.NavigableMap;
import java.util.Optional;
import java.util.TreeMap;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.vision.Camera;

public class PoseEstimatorSubsystem extends SubsystemBase {

  private final DriveSubsystem driveSubsystem;
  
  // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
  // you trust your various sensors. Smaller numbers will cause the filter to
  // "trust" the estimate from that particular component more than the others. 
  // This in turn means the particualr component will have a stronger influence
  // on the final pose estimate.

  /**
   * Standard deviations of model states. Increase these numbers to trust your model's state estimates less. This
   * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then meters.
   */
  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
  
  /**
   * Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
   * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
   */
  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

  private final SwerveDrivePoseEstimator poseEstimator;

  private final Field2d field2d = new Field2d();

  public PoseEstimatorSubsystem(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;

    SmartDashboard.putNumber("Shoot Angle", 30);


    //-74 at rest
    // @45  = -48

    ShuffleboardTab tab = Shuffleboard.getTab("Vision");
    
    poseEstimator =  new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics,
        driveSubsystem.getGyroscopeRotation(),
        driveSubsystem.getModulePositions(),
        new Pose2d(new Translation2d(0, 2), new Rotation2d(180)),
        stateStdDevs,
        visionMeasurementStdDevs);
    
    tab.addString("Pose", this::getFomattedPose).withPosition(0, 0).withSize(2, 0);
    tab.add("Field", field2d).withPosition(2, 0).withSize(6, 4);
  }

  void resetPosition(Rotation2d rotation, Pose2d pose ) {
    poseEstimator.resetPosition(rotation, driveSubsystem.getModulePositions(), pose);
  }

  @Override
  public void periodic() {
    // for(frc.robot.subsystems.vision.Camera.PoseInstance p: Camera.DRIVE_CAMERA.getLatest()) {
    //     poseEstimator.addVisionMeasurement(p.getPose(), p.getTimestamp());
    // }

    Optional<EstimatedRobotPose> estimatedPose = Camera.SHOOT_CAMERA.getEstimatedGlobalPose();
    if(estimatedPose.isPresent()) {
      poseEstimator.addVisionMeasurement(estimatedPose.get().estimatedPose.toPose2d(), estimatedPose.get().timestampSeconds);
    }

    // Update pose estimator with driveSubsystem sensors
    poseEstimator.update(
      driveSubsystem.getGyroscopeRotation(),
      driveSubsystem.getModulePositions());

    field2d.setRobotPose(getCurrentPose());
  }

  private String getFomattedPose() {
    return getFomattedPose(getCurrentPose());
  }

  private String getFomattedPose(Pose2d pose) {

    return String.format("(%.2f, %.2f) %.2f degrees", 
        pose.getX(), 
        pose.getY(),
        pose.getRotation().getDegrees());
  }

  public Pose2d getCurrentPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the current pose to the specified pose. This should ONLY be called
   * when the robot's position on the field is known, like at the beginning of
   * a match.
   * @param newPose new pose
   */
  public void setCurrentPose(Pose2d newPose) {
    poseEstimator.resetPosition(
      driveSubsystem.getGyroscopeRotation(),
      driveSubsystem.getModulePositions(),
      newPose);
  }

  /**
   * Resets the position on the field to 0,0 0-degrees, with forward being downfield. This resets
   * what "forward" is for field oriented driving.
   */
  public void resetFieldPosition() {
    setCurrentPose(new Pose2d());
  }

  public double getVerticalShootAngle() {

    double angle = SmartDashboard.getNumber("Shoot Angle", 30);
    return angle;
    /* 
    Translation3d target = getShootTarget();
    double distance = getSpeakerDistance();

    return Math.atan(Constants.VisionConstants.TARGET_HEIGHT_DIFF/distance);
    */
  }

  public double getSpeakerDistance() {
    Pose2d pose  = getCurrentPose();
    Translation3d target = getShootTarget();
     double xdiff = Math.abs(pose.getX() - target.getX());
     double ydiff = Math.abs(pose.getY() - target.getY());

    return Math.sqrt(ydiff*ydiff + xdiff*xdiff );
  }

  public double getHorizontalShootAngle() {
    Pose2d pose  = getCurrentPose();
    Translation3d target = getShootTarget();
    double xdiff = Math.abs(pose.getX() - target.getX());
    double ydiff = Math.abs(pose.getY() - target.getY());

    double angle = Math.atan( ydiff / xdiff );

    if (blueAlliance()) {
      if (pose.getY() > target.getY()) {
        return 270 - angle;
      } else {
        return 90 + angle;
      }
    }
    // Red alliance
    if (pose.getY() > target.getY()) {
      return 270 + angle;
    }
    return 90 - angle;
}

  private Translation3d getShootTarget() {
    if ( blueAlliance()) {
      return Constants.VisionConstants.BLUE_SPEAKER_TARGET;
    }
    return Constants.VisionConstants.RED_SPEAKER_TARGET;
  }

  public boolean blueAlliance() {
      return DriverStation.getAlliance().get() == Alliance.Blue;
  }

}