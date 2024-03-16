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

public class BlueRight extends GenAutoCommand {
  public static Pose2d StartPose = new Pose2d(0.890000, 4.450000, new Rotation2d(Units.degreesToRadians(-60.000000)));

  @Override
  public Pose2d getStartingPose() {
    return StartPose;
  }
  public static String NAME = "BlueRight";

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
      new Pose2d(0.8938335058664806, 4.452607704941355,  new Rotation2d(Units.degreesToRadians(-59.99999999999999))),
      new ArrayList<Translation2d>(),
      new Pose2d(8.528240247683364, 0.37, new Rotation2d(Units.degreesToRadians(0.0)))
  );
  return trajectoryCommandFactory.createTrajectoryCommand(trajectory);
}

public static Command buildDriveIn1(DriveSubsystem driveSubsystem, TrajectoryCommandFactory trajectoryCommandFactory) {
  Trajectory trajectory = trajectoryCommandFactory.createTrajectory(
      new Pose2d(8.28, 0.79,  new Rotation2d(Units.degreesToRadians(0.0))),
      List.of(
        new Translation2d(5.124431127610109, 1.4897778107624486)
      ),
      new Pose2d(3.4545606733344716, 2.904705461919659, new Rotation2d(Units.degreesToRadians(-47.3532968661083)))
  );
  return trajectoryCommandFactory.createTrajectoryCommand(trajectory);
}

public static Command buildDriveOut2(DriveSubsystem driveSubsystem, TrajectoryCommandFactory trajectoryCommandFactory) {
  Trajectory trajectory = trajectoryCommandFactory.createTrajectory(
      new Pose2d(3.4545606733344716, 2.904705461919659,  new Rotation2d(Units.degreesToRadians(-47.3532968661083))),
      List.of(
        new Translation2d(5.523478748697167, 1.3301587623276245)
      ),
      new Pose2d(8.3, 2.436512808166183, new Rotation2d(Units.degreesToRadians(23.025492008528147)))
  );
  return trajectoryCommandFactory.createTrajectoryCommand(trajectory);
}

public static Command buildDriveIn2(DriveSubsystem driveSubsystem, TrajectoryCommandFactory trajectoryCommandFactory) {
  Trajectory trajectory = trajectoryCommandFactory.createTrajectory(
      new Pose2d(8.3, 2.436512808166183,  new Rotation2d(Units.degreesToRadians(23.025492008528147))),
      List.of(
        new Translation2d(5.430367637110186, 1.5163809855015853)
      ),
      new Pose2d(3.4545606733344716, 2.904705461919659, new Rotation2d(Units.degreesToRadians(-44.49247853130129)))
  );
  return trajectoryCommandFactory.createTrajectoryCommand(trajectory);
}

}
