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

public class DriveBack {
  public static Pose2d StartPose = new Pose2d(1.000000, 6.793571, new Rotation2d(Units.degreesToRadians(60.000000)));

  public static Command buildAuto(DriveSubsystem driveSubsystem, TrajectoryCommandFactory trajectoryCommandFactory) {
    return build1Meter(driveSubsystem, trajectoryCommandFactory);
  }

  
public static Command build1Meter(DriveSubsystem driveSubsystem, TrajectoryCommandFactory trajectoryCommandFactory) {
  Trajectory trajectory = trajectoryCommandFactory.createTrajectory(
      new Pose2d(1.8206638612560146, 7.338203244401553,  new Rotation2d(Units.degreesToRadians(59.99999999999999))),
      new ArrayList<Translation2d>(),
      new Pose2d(1.8206638612560146, 7.338203244401553, new Rotation2d(Units.degreesToRadians(120.0)))
  );
  return trajectoryCommandFactory.createTrajectoryCommand(trajectory);
}

}
