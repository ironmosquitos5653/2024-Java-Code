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
  public static Pose2d StartPose = new Pose2d(1.5, 5.551427198444411,  new Rotation2d(Units.degreesToRadians(0)));

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
      .andThen(CommandRegistry.getCommand("ShooterOff"))
      .andThen(buildBM4(driveSubsystem, trajectoryCommandFactory));
  }

  
public static Command buildBM1(DriveSubsystem driveSubsystem, TrajectoryCommandFactory trajectoryCommandFactory) {
  Trajectory trajectory = trajectoryCommandFactory.createTrajectory(
      new Pose2d(1.5, 5.551427198444411,  new Rotation2d(Units.degreesToRadians(0))),
      List.of(
        new Translation2d(2, 4.508629428714312)
      ),
      new Pose2d(3, 4.18629428714312, new Rotation2d(Units.degreesToRadians(0)))
  );
  return trajectoryCommandFactory.createTrajectoryCommand(trajectory);
}

public static Command buildBMShoot(DriveSubsystem driveSubsystem, TrajectoryCommandFactory trajectoryCommandFactory) {
  Trajectory trajectory = trajectoryCommandFactory.createTrajectory(
      new Pose2d(3, 4.11,  new Rotation2d(Units.degreesToRadians(0))),
      new ArrayList<Translation2d>(),
      new Pose2d(2, 5.51, new Rotation2d(Units.degreesToRadians(0)))
  );
  return trajectoryCommandFactory.createTrajectoryCommand(trajectory);
}

public static Command buildBM2(DriveSubsystem driveSubsystem, TrajectoryCommandFactory trajectoryCommandFactory) {
  Trajectory trajectory = trajectoryCommandFactory.createTrajectory(
      new Pose2d(2, 5.51,  new Rotation2d(Units.degreesToRadians(0))),
      new ArrayList<Translation2d>(),
      new Pose2d(3.1, 5.820158783741731, new Rotation2d(Units.degreesToRadians(0)))
  );
  return trajectoryCommandFactory.createTrajectoryCommand(trajectory);
}

public static Command buildBM3(DriveSubsystem driveSubsystem, TrajectoryCommandFactory trajectoryCommandFactory) {
  Trajectory trajectory = trajectoryCommandFactory.createTrajectory(
      new Pose2d(3.1, 5.820158783741731,  new Rotation2d(Units.degreesToRadians(0))),
      List.of(
        new Translation2d(2, 6.390190962200107)
      ),
      new Pose2d(3, 6.968573209873693, new Rotation2d(Units.degreesToRadians(33)))
  );
  return trajectoryCommandFactory.createTrajectoryCommand(trajectory);
}

public static Command buildBM4(DriveSubsystem driveSubsystem, TrajectoryCommandFactory trajectoryCommandFactory) {
  Trajectory trajectory = trajectoryCommandFactory.createTrajectory(
      new Pose2d(3, 7,  new Rotation2d(Units.degreesToRadians(33))),
      new ArrayList<Translation2d>(),
      new Pose2d(4.5, 8, new Rotation2d(Units.degreesToRadians(0)))
  );
  return trajectoryCommandFactory.createTrajectoryCommand(trajectory);
}

}
