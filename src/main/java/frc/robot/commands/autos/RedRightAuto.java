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

public class RedRightAuto {
  public static Pose2d StartPose = new Pose2d(15.591000, 6.720000, new Rotation2d(Units.degreesToRadians(120.000000)));

  public static String NAME = "RedRightAuto";

  public static Command buildAuto(DriveSubsystem driveSubsystem, TrajectoryCommandFactory trajectoryCommandFactory) {
    return CommandRegistry.getCommand("Lift 53")
      .andThen(CommandRegistry.getCommand("ShooterOn"))
      .andThen(CommandRegistry.getCommand("Shoot"))
      .andThen(
        CommandRegistry.getCommand("IntakeOn")
        .alongWith(buildBLDriveOut(driveSubsystem, trajectoryCommandFactory)))
      .andThen(CommandRegistry.getCommand("Lift B"))
      .andThen(buildBLDriveIn(driveSubsystem, trajectoryCommandFactory))
      .andThen(CommandRegistry.getCommand("Shoot"))
      .andThen(
        CommandRegistry.getCommand("IntakeOn")
        .alongWith(buildBLDriveOut2(driveSubsystem, trajectoryCommandFactory)))
      .andThen(CommandRegistry.getCommand("Lift B"))
      .andThen(buildBLDriveIn2(driveSubsystem, trajectoryCommandFactory))
      .andThen(CommandRegistry.getCommand("Shoot"))
      .andThen(CommandRegistry.getCommand("ShooterOff"));
  }

  
public static Command buildBLDriveOut(DriveSubsystem driveSubsystem, TrajectoryCommandFactory trajectoryCommandFactory) {
  Trajectory trajectory = trajectoryCommandFactory.createTrajectory(
      new Pose2d(15.589836781429012, 6.717131356769391,  new Rotation2d(Units.degreesToRadians(120.0))),
      List.of(
        new Translation2d(14.443242527338867, 7.9),
        new Translation2d(11.19455880741679, 7.577077047337)
      ),
      new Pose2d(7.665638986195322, 7.271972574045404, new Rotation2d(Units.degreesToRadians(179.4031905487708)))
  );
  return trajectoryCommandFactory.createTrajectoryCommand(trajectory);
}

public static Command buildBLDriveIn(DriveSubsystem driveSubsystem, TrajectoryCommandFactory trajectoryCommandFactory) {
  Trajectory trajectory = trajectoryCommandFactory.createTrajectory(
      new Pose2d(8.213413746782413, 7.4815275261628225,  new Rotation2d(Units.degreesToRadians(179.4031905487708))),
      new ArrayList<Translation2d>(),
      new Pose2d(12.780680858908157, 5.809410905614693, new Rotation2d(Units.degreesToRadians(164.54)))
  );
  return trajectoryCommandFactory.createTrajectoryCommand(trajectory);
}

public static Command buildBLDriveOut2(DriveSubsystem driveSubsystem, TrajectoryCommandFactory trajectoryCommandFactory) {
  Trajectory trajectory = trajectoryCommandFactory.createTrajectory(
      new Pose2d(12.780680858908157, 5.809410905614693,  new Rotation2d(Units.degreesToRadians(164.54))),
      new ArrayList<Translation2d>(),
      new Pose2d(7.6, 5.4, new Rotation2d(Units.degreesToRadians(-135.80692945533443)))
  );
  return trajectoryCommandFactory.createTrajectoryCommand(trajectory);
}

public static Command buildBLDriveIn2(DriveSubsystem driveSubsystem, TrajectoryCommandFactory trajectoryCommandFactory) {
  Trajectory trajectory = trajectoryCommandFactory.createTrajectory(
      new Pose2d(8.261188507369502, 5.761636145027605,  new Rotation2d(Units.degreesToRadians(-135.80692945533443))),
      List.of(
        new Translation2d(10, 7)
      ),
      new Pose2d(12.780680858908157, 5.809410905614693, new Rotation2d(Units.degreesToRadians(164.54)))
  );
  return trajectoryCommandFactory.createTrajectoryCommand(trajectory);
}

}
