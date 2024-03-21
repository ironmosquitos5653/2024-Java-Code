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

public class BlueShoot {
  public static Pose2d StartPose = new Pose2d(.6, 0, new Rotation2d(Units.degreesToRadians(0)));

  public static String NAME = "BlueShoot";

  public static Command buildAuto(DriveSubsystem driveSubsystem, TrajectoryCommandFactory trajectoryCommandFactory) {
    return new WaitCommand(5)
      .andThen(CommandRegistry.getCommand("ShooterOn"))
      .andThen(CommandRegistry.getCommand("Lift BS"))
      .andThen(buildDriveOut1(driveSubsystem, trajectoryCommandFactory))
      .andThen(CommandRegistry.getCommand("Shoot"))
      .andThen(CommandRegistry.getCommand("ShooterOff"));
  }

public static Command buildDriveOut1(DriveSubsystem driveSubsystem, TrajectoryCommandFactory trajectoryCommandFactory) {
  Trajectory trajectory = trajectoryCommandFactory.createTrajectory(
      new Pose2d(.6, 0,  new Rotation2d(Units.degreesToRadians(0))),
      new ArrayList<Translation2d>(),
      new Pose2d(3.2, 0, new Rotation2d(Units.degreesToRadians(-55)))
  );
  return trajectoryCommandFactory.createTrajectoryCommand(trajectory);
 }
}
