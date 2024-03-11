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

public class BlueLeftAuto {
  public static Pose2d StartPose = new Pose2d(0.950000, 6.720000, new Rotation2d(Units.degreesToRadians(60.000000)));

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
      new Pose2d(8.375361013804678, 7.471972574045404,  new Rotation2d(Units.degreesToRadians(59.99999999999999))),
      List.of(
        new Translation2d(2.0977574726611325, 7.577077047337),
        new Translation2d(5.346441192583211, 7.577077047337)
      ),
      new Pose2d(8.375361013804678, 7.471972574045404, new Rotation2d(Units.degreesToRadians(0.5968094512292037)))
  );
  return trajectoryCommandFactory.createTrajectoryCommand(trajectory);
}

public static Command buildBLDriveIn(DriveSubsystem driveSubsystem, TrajectoryCommandFactory trajectoryCommandFactory) {
  Trajectory trajectory = trajectoryCommandFactory.createTrajectory(
      new Pose2d(3.7603191410918435, 5.809410905614693,  new Rotation2d(Units.degreesToRadians(0.5968094512292037))),
      new ArrayList<Translation2d>(),
      new Pose2d(3.7603191410918435, 5.809410905614693, new Rotation2d(Units.degreesToRadians(15.46)))
  );
  return trajectoryCommandFactory.createTrajectoryCommand(trajectory);
}

public static Command buildBLDriveOut2(DriveSubsystem driveSubsystem, TrajectoryCommandFactory trajectoryCommandFactory) {
  Trajectory trajectory = trajectoryCommandFactory.createTrajectory(
      new Pose2d(8.279811492630499, 5.761636145027605,  new Rotation2d(Units.degreesToRadians(15.46))),
      new ArrayList<Translation2d>(),
      new Pose2d(8.279811492630499, 5.761636145027605, new Rotation2d(Units.degreesToRadians(-44.19307054466558)))
  );
  return trajectoryCommandFactory.createTrajectoryCommand(trajectory);
}

public static Command buildBLDriveIn2(DriveSubsystem driveSubsystem, TrajectoryCommandFactory trajectoryCommandFactory) {
  Trajectory trajectory = trajectoryCommandFactory.createTrajectory(
      new Pose2d(3.7603191410918435, 5.809410905614693,  new Rotation2d(Units.degreesToRadians(-44.19307054466558))),
      new ArrayList<Translation2d>(),
      new Pose2d(3.7603191410918435, 5.809410905614693, new Rotation2d(Units.degreesToRadians(15.46)))
  );
  return trajectoryCommandFactory.createTrajectoryCommand(trajectory);
}

}
