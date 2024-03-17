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

  public static String NAME = "BlueMiddle";

  public static Command buildAuto(DriveSubsystem driveSubsystem, TrajectoryCommandFactory trajectoryCommandFactory) {
    return CommandRegistry.getCommand("Lift 53")
      .andThen(new WaitCommand(0.75))
      .andThen(CommandRegistry.getCommand("ShooterOn"))
      .andThen(CommandRegistry.getCommand("Shoot"))
      .andThen(CommandRegistry.getCommand("Lift 40"))
      .andThen(
        CommandRegistry.getCommand("IntakeOn")
        .alongWith(buildBM1(driveSubsystem, trajectoryCommandFactory)))
      .andThen(buildBMShoot(driveSubsystem, trajectoryCommandFactory))
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
      new Pose2d(1.4957954892638068, 5.551427198444411,  new Rotation2d(Units.degreesToRadians(0.0))),
      new ArrayList<Translation2d>(),
      new Pose2d(2.766604120880384, 4.108629428714312, new Rotation2d(Units.degreesToRadians(0.0)))
  );
  return trajectoryCommandFactory.createTrajectoryCommand(trajectory);
}

public static Command buildBMShoot(DriveSubsystem driveSubsystem, TrajectoryCommandFactory trajectoryCommandFactory) {
  Trajectory trajectory = trajectoryCommandFactory.createTrajectory(
      new Pose2d(2.77, 4.11,  new Rotation2d(Units.degreesToRadians(0.0))),
      new ArrayList<Translation2d>(),
      new Pose2d(2.6232798391191166, 4.11, new Rotation2d(Units.degreesToRadians(-28.80534954987098)))
  );
  return trajectoryCommandFactory.createTrajectoryCommand(trajectory);
}

public static Command buildBM2(DriveSubsystem driveSubsystem, TrajectoryCommandFactory trajectoryCommandFactory) {
  Trajectory trajectory = trajectoryCommandFactory.createTrajectory(
      new Pose2d(2.6232798391191166, 4.11,  new Rotation2d(Units.degreesToRadians(-28.80534954987098))),
      List.of(
        new Translation2d(2.0386542591267003, 5.214768974992495)
      ),
      new Pose2d(2.71, 5.520158783741731, new Rotation2d(Units.degreesToRadians(0.0)))
  );
  return trajectoryCommandFactory.createTrajectoryCommand(trajectory);
}

public static Command buildBM3(DriveSubsystem driveSubsystem, TrajectoryCommandFactory trajectoryCommandFactory) {
  Trajectory trajectory = trajectoryCommandFactory.createTrajectory(
      new Pose2d(2.71, 5.520158783741731,  new Rotation2d(Units.degreesToRadians(0.0))),
      List.of(
        new Translation2d(2.0852979887777954, 6.390190962200107)
      ),
      new Pose2d(2.887570138776641, 6.968573209873693, new Rotation2d(Units.degreesToRadians(33.0)))
  );
  return trajectoryCommandFactory.createTrajectoryCommand(trajectory);
}

}
