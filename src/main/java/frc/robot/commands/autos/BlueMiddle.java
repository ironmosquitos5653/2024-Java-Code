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

public class BlueMiddle {
  public static Pose2d StartPose = new Pose2d(1.495795, 5.551427, new Rotation2d(Units.degreesToRadians(0.000000)));

  public static Command buildAuto(DriveSubsystem driveSubsystem, TrajectoryCommandFactory trajectoryCommandFactory) {
    return CommandRegistry.getCommand("Lift 53")
      .andThen(new WaitCommand(0.75))
      .andThen(CommandRegistry.getCommand("ShooterOn"))
      .andThen(CommandRegistry.getCommand("Shoot"))
      .andThen(CommandRegistry.getCommand("Lift 40"))
      .andThen(
        CommandRegistry.getCommand("IntakeOn")
        .alongWith(buildBM1(driveSubsystem, trajectoryCommandFactory)))
      .andThen(CommandRegistry.getCommand("Shoot"))
      .andThen(CommandRegistry.getCommand("Lift 40"))
      .andThen(
        CommandRegistry.getCommand("IntakeOn")
        .alongWith(buildBM2(driveSubsystem, trajectoryCommandFactory)))
      .andThen(CommandRegistry.getCommand("Shoot"))
      .andThen(
        CommandRegistry.getCommand("IntakeOn")
        .alongWith(buildBM3(driveSubsystem, trajectoryCommandFactory)))
      .andThen(CommandRegistry.getCommand("Shoot"))
      .andThen(CommandRegistry.getCommand("ShooterOff"));
  }

  
public static Command buildBM1(DriveSubsystem driveSubsystem, TrajectoryCommandFactory trajectoryCommandFactory) {
  Trajectory trajectory = trajectoryCommandFactory.createTrajectory(
      new Pose2d(2.8334887857023094, 4.165959141418819,  new Rotation2d(Units.degreesToRadians(0.0))),
      new ArrayList<Translation2d>(),
      new Pose2d(2.8334887857023094, 4.165959141418819, new Rotation2d(Units.degreesToRadians(-32.12499844038754)))
  );
  return trajectoryCommandFactory.createTrajectoryCommand(trajectory);
}

public static Command buildBM2(DriveSubsystem driveSubsystem, TrajectoryCommandFactory trajectoryCommandFactory) {
  Trajectory trajectory = trajectoryCommandFactory.createTrajectory(
      new Pose2d(2.71, 5.520158783741731,  new Rotation2d(Units.degreesToRadians(-32.12499844038754))),
      new ArrayList<Translation2d>(),
      new Pose2d(2.71, 5.520158783741731, new Rotation2d(Units.degreesToRadians(0.0)))
  );
  return trajectoryCommandFactory.createTrajectoryCommand(trajectory);
}

public static Command buildBM3(DriveSubsystem driveSubsystem, TrajectoryCommandFactory trajectoryCommandFactory) {
  Trajectory trajectory = trajectoryCommandFactory.createTrajectory(
      new Pose2d(2.8812635462893987, 6.936895255470003,  new Rotation2d(Units.degreesToRadians(0.0))),
      new ArrayList<Translation2d>(),
      new Pose2d(2.8812635462893987, 6.936895255470003, new Rotation2d(Units.degreesToRadians(28.43476314954997)))
  );
  return trajectoryCommandFactory.createTrajectoryCommand(trajectory);
}

}
