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

public class Test {
  public static Pose2d StartPose = new Pose2d(1.763334, 6.392263, new Rotation2d(Units.degreesToRadians(-90.000000)));

  public static Command buildAuto(DriveSubsystem driveSubsystem, TrajectoryCommandFactory trajectoryCommandFactory) {
    return buildRedLeftTest(driveSubsystem, trajectoryCommandFactory);
  }

  
public static Command buildRedLeftTest(DriveSubsystem driveSubsystem, TrajectoryCommandFactory trajectoryCommandFactory) {
  Trajectory trajectory = trajectoryCommandFactory.createTrajectory(
      new Pose2d(1.763334148551507, 5.990954995845633,  new Rotation2d(Units.degreesToRadians(-90.0))),
      new ArrayList<Translation2d>(),
      new Pose2d(1.763334148551507, 5.990954995845633, new Rotation2d(Units.degreesToRadians(-90.49821161261363)))
  );
  return trajectoryCommandFactory.createTrajectoryCommand(trajectory);
}

}
