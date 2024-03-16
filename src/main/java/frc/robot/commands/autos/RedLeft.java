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
      .andThen(CommandRegistry.getCommand("Lift 35"))
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
      new Pose2d(15.64716649413352, 4.452607704941355,  new Rotation2d(Units.degreesToRadians(-120.0))),
      new ArrayList<Translation2d>(),
      new Pose2d(8.012759752316637, 0.7739511397354721, new Rotation2d(Units.degreesToRadians(180.0)))
  );
  return trajectoryCommandFactory.createTrajectoryCommand(trajectory);
}

public static Command buildDriveIn1(DriveSubsystem driveSubsystem, TrajectoryCommandFactory trajectoryCommandFactory) {
  Trajectory trajectory = trajectoryCommandFactory.createTrajectory(
      new Pose2d(8.261000000000001, 0.79,  new Rotation2d(Units.degreesToRadians(180.0))),
      List.of(
        new Translation2d(11.416568872389892, 1.4897778107624486)
      ),
      new Pose2d(13.086439326665529, 2.904705461919659, new Rotation2d(Units.degreesToRadians(-132.6467031338917)))
  );
  return trajectoryCommandFactory.createTrajectoryCommand(trajectory);
}

public static Command buildDriveOut2(DriveSubsystem driveSubsystem, TrajectoryCommandFactory trajectoryCommandFactory) {
  Trajectory trajectory = trajectoryCommandFactory.createTrajectory(
      new Pose2d(13.086439326665529, 2.904705461919659,  new Rotation2d(Units.degreesToRadians(-132.6467031338917))),
      List.of(
        new Translation2d(11.017521251302833, 1.3301587623276245)
      ),
      new Pose2d(8.241, 2.436512808166183, new Rotation2d(Units.degreesToRadians(156.97450799147185)))
  );
  return trajectoryCommandFactory.createTrajectoryCommand(trajectory);
}

public static Command buildDriveIn2(DriveSubsystem driveSubsystem, TrajectoryCommandFactory trajectoryCommandFactory) {
  Trajectory trajectory = trajectoryCommandFactory.createTrajectory(
      new Pose2d(8.241, 2.436512808166183,  new Rotation2d(Units.degreesToRadians(156.97450799147185))),
      List.of(
        new Translation2d(11.110632362889813, 1.5163809855015853)
      ),
      new Pose2d(13.086439326665529, 2.904705461919659, new Rotation2d(Units.degreesToRadians(-135.5075214686987)))
  );
  return trajectoryCommandFactory.createTrajectoryCommand(trajectory);
}

}
