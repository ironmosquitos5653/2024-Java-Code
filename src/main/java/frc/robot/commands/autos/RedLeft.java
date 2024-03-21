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

public class RedLeft {
  public static Pose2d StartPose = new Pose2d(15.651000, 4.450000, new Rotation2d(Units.degreesToRadians(-120.000000)));

  public static String NAME = "RedLeft";

  public static Command buildAuto(DriveSubsystem driveSubsystem, TrajectoryCommandFactory trajectoryCommandFactory) {
    return CommandRegistry.getCommand("ShooterOn")
      .andThen(CommandRegistry.getCommand("Lift 53"))
      .andThen(new WaitCommand(0.75))
      .andThen(CommandRegistry.getCommand("Shoot"))
      .andThen(
        CommandRegistry.getCommand("IntakeOn")
        .alongWith(buildDriveOut1(driveSubsystem, trajectoryCommandFactory)))
      .andThen(CommandRegistry.getCommand("Lift PP"))
      .andThen(buildDriveIn1(driveSubsystem, trajectoryCommandFactory))
      .andThen(CommandRegistry.getCommand("Shoot"))
      .andThen(
        CommandRegistry.getCommand("IntakeOn")
        .alongWith(buildDriveOut2(driveSubsystem, trajectoryCommandFactory)))
      .andThen(buildDriveIn2(driveSubsystem, trajectoryCommandFactory))
      .andThen(CommandRegistry.getCommand("Shoot"))
      .andThen(CommandRegistry.getCommand("ShooterOff"));
  }

  
public static Command buildDriveOut1(DriveSubsystem driveSubsystem, TrajectoryCommandFactory trajectoryCommandFactory) {
  Trajectory trajectory = trajectoryCommandFactory.createTrajectory(
      new Pose2d(15.6, 4.452607704941355,  new Rotation2d(Units.degreesToRadians(-120))),
      new ArrayList<Translation2d>(),
      new Pose2d(8, 0.37, new Rotation2d(Units.degreesToRadians(180)))
  );
  return trajectoryCommandFactory.createTrajectoryCommand(trajectory);
}

public static Command buildDriveIn1(DriveSubsystem driveSubsystem, TrajectoryCommandFactory trajectoryCommandFactory) {
  Trajectory trajectory = trajectoryCommandFactory.createTrajectory(
      new Pose2d(8, 0.79,  new Rotation2d(Units.degreesToRadians(180))),
      List.of(
        new Translation2d(11.4, 1.3301587623276245)
      ),
      new Pose2d(13.1, 2.904705461919659, new Rotation2d(Units.degreesToRadians(-133)))
  );
  return trajectoryCommandFactory.createTrajectoryCommand(trajectory);
}

public static Command buildDriveOut2(DriveSubsystem driveSubsystem, TrajectoryCommandFactory trajectoryCommandFactory) {
  Trajectory trajectory = trajectoryCommandFactory.createTrajectory(
      new Pose2d(13.1, 2.904705461919659,  new Rotation2d(Units.degreesToRadians(-133))),
      List.of(
        new Translation2d(11.4, 1.3301587623276245)
      ),
      new Pose2d(8, 2.436512808166183, new Rotation2d(Units.degreesToRadians(157)))
  );
  return trajectoryCommandFactory.createTrajectoryCommand(trajectory);
}

public static Command buildDriveIn2(DriveSubsystem driveSubsystem, TrajectoryCommandFactory trajectoryCommandFactory) {
  Trajectory trajectory = trajectoryCommandFactory.createTrajectory(
      new Pose2d(8, 2.436512808166183,  new Rotation2d(Units.degreesToRadians(157))),
      List.of(
        new Translation2d(11.4, 1.5163809855015853)
      ),
      new Pose2d(13.1, 2.904705461919659, new Rotation2d(Units.degreesToRadians(-136)))
  );
  return trajectoryCommandFactory.createTrajectoryCommand(trajectory);
}

}
