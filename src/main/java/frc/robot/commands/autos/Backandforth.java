package frc.robot.commands.autos;
import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.TrajectoryCommandFactory;
import frc.robot.subsystems.DriveSubsystem;
import java.util.List;

public class Backandforth {
  public static Pose2d StartPose = new Pose2d(0.750509, 6.889120, new Rotation2d(Units.degreesToRadians(59.036243)));

  public static String NAME = "Back and forth";

  public static Command buildAuto(DriveSubsystem driveSubsystem, TrajectoryCommandFactory trajectoryCommandFactory) {
    return buildFromSpeaker(driveSubsystem, trajectoryCommandFactory)
      .andThen(buildToSpeaker(driveSubsystem, trajectoryCommandFactory));
  }

  
public static Command buildFromSpeaker(DriveSubsystem driveSubsystem, TrajectoryCommandFactory trajectoryCommandFactory) {
  Trajectory trajectory = trajectoryCommandFactory.createTrajectory(
      new Pose2d(0.7505092241052124, 6.8891204948829134,  new Rotation2d(Units.degreesToRadians(59.03624346792646))),
      new ArrayList<Translation2d>(),
      new Pose2d(5.4, 6.698021452534556, new Rotation2d(Units.degreesToRadians(0.0)))
  );
  return trajectoryCommandFactory.createTrajectoryCommand(trajectory);
}

public static Command buildToSpeaker(DriveSubsystem driveSubsystem, TrajectoryCommandFactory trajectoryCommandFactory) {
  Trajectory trajectory = trajectoryCommandFactory.createTrajectory(
      new Pose2d(5.4, 6.7,  new Rotation2d(Units.degreesToRadians(0.0))),
      new ArrayList<Translation2d>(),
      new Pose2d(0.75, 6.89, new Rotation2d(Units.degreesToRadians(59.11847248521549)))
  );
  return trajectoryCommandFactory.createTrajectoryCommand(trajectory);
}

}
