package frc.robot.subsystems;


import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
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

  private static boolean visionOn = false;



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
        new Pose2d(new Translation2d(0, 0), new Rotation2d(0)),
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
    getVerticalShootAngle();
    getSpeakerDistance();
    getHorizontalShootAngle();

    // for(frc.robot.subsystems.vision.Camera.PoseInstance p: Camera.DRIVE_CAMERA.getLatest()) {
    //     poseEstimator.addVisionMeasurement(p.getPose(), p.getTimestamp());
    // }

    Optional<EstimatedRobotPose> estimatedPose = Camera.SHOOT_CAMERA.getEstimatedGlobalPose();
    if(estimatedPose.isPresent()) {
      PhotonPipelineResult result = Camera.SHOOT_CAMERA.getPhotonCamera().getLatestResult();
      if (result.hasTargets()) {
       PhotonTrackedTarget target = result.getBestTarget();
       Optional<Pose3d> tagPose = Camera.getFieldLayout().getTagPose(target.getFiducialId());
        if (tagPose.isPresent() && visionOn) {
            Pose2d p = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), tagPose.get(), Camera.SHOOT_CAMERA.getRobotToCamera().inverse()).toPose2d();
          poseEstimator.addVisionMeasurement(p, estimatedPose.get().timestampSeconds);
        }
      }
    }

    // Update pose estimator with driveSubsystem sensors
    poseEstimator.update(
      driveSubsystem.getGyroscopeRotation(),
      driveSubsystem.getModulePositions());

    field2d.setRobotPose(getCurrentPose());
  }

  public static void setVisionEnabled(boolean enable){
    visionOn = enable;
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
    return voodooMath();
    // double distance = getSpeakerDistance();

    // double angle = Units.radiansToDegrees(Math.atan(Constants.VisionConstants.TARGET_HEIGHT_DIFF/distance));
    // SmartDashboard.putNumber("Vertical Angle", angle);
    // return angle;
  }

  public double voodooMath() {
    

    //  ***********************************************************
    //  Increase lower bound to raise shot from far away
     //  ***********************************************************
    double lowerBound = 38;
    double upperBound = 53;
    double angleRange = upperBound - lowerBound;

    double upperDistance = 1.368;
    double lowerDistance = 3.614;
    double distanceRange = upperDistance - lowerDistance;

    double distance = getSpeakerDistance();

    double distanceOffset = distance - lowerDistance;
    double distancePercent = distanceOffset/distanceRange;

    double angle= distancePercent * angleRange + lowerBound;
    return angle;
  }

  public double getSpeakerDistance() {
    Pose2d pose  = getCurrentPose();
    Translation3d target = getShootTarget();
     double xdiff = Math.abs(pose.getX() - target.getX());
     double ydiff = Math.abs(pose.getY() - target.getY());

    double distance = Math.sqrt(ydiff*ydiff + xdiff*xdiff );
    SmartDashboard.putNumber("* distance", distance);
    SmartDashboard.putNumber("photon distance", PhotonUtils.getDistanceToPose(pose, new Pose3d(target, new Rotation3d()).toPose2d()));
    return distance;
  }

  public double   getHorizontalShootAngle() {
    Pose2d pose  = getCurrentPose();
    Translation3d target = getShootTarget();
    double xdiff = Math.abs(pose.getX() - target.getX());
    double ydiff = Math.abs(pose.getY() - target.getY());
    

    double angle = Units.radiansToDegrees(Math.atan( ydiff / xdiff ));
    SmartDashboard.putString("Shoot Target", target.getX() + " - " + target.getY());
    SmartDashboard.putNumber("*horizontal angle", angle);
    if (blueAlliance()) {
      if (pose.getY() > target.getY()) {
        return angle;
      } else {
        return - angle;
      }
    }

    if (pose.getY() > target.getY()) {
        return 180 - angle;
      } 

    return angle - 180;
}

  private Translation3d getShootTarget() {
    if ( blueAlliance()) {
      return Constants.VisionConstants.BLUE_SPEAKER_TARGET;
    }
    return Constants.VisionConstants.RED_SPEAKER_TARGET;
  }

  private String getShootTargetPose() {
    return getShootTarget().getX() + " - " + getShootTarget().getY();
  }

  public boolean blueAlliance() {
      return DriverStation.getAlliance().get() == Alliance.Blue;
  }

}