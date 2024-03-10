// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
import frc.robot.subsystems.vision.Camera;

public class Test1Red {
  public static Pose2d StartPose = new Pose2d(Camera.getFieldLayout().getFieldLength() - .95, 6.72, new Rotation2d(Units.degreesToRadians(120)));

  public static Command buildAuto(DriveSubsystem driveSubsystem, TrajectoryCommandFactory trajectoryCommandFactory) {
      return CommandRegistry.getCommand("Lift 53")
      .andThen(new WaitCommand(.5))
      //.andThen(CommandRegistry.getCommand("ShooterOn"))
      //.andThen(CommandRegistry.getCommand("Shoot"))
      .andThen(buildPathOut(driveSubsystem, trajectoryCommandFactory))
      .andThen(CommandRegistry.getCommand("ShooterOff"));

  }

  public static Command buildPathOut(DriveSubsystem driveSubsystem, TrajectoryCommandFactory trajectoryCommandFactory) {
            Trajectory driveIn3Ball = trajectoryCommandFactory.createTrajectory(
            new Pose2d(Camera.getFieldLayout().getFieldLength() - .95, 6.72, new Rotation2d(Units.degreesToRadians(120))),
            new ArrayList<Translation2d>(),
            new Pose2d(Camera.getFieldLayout().getFieldLength() - 1.6868945316121644, 7.462417621927986, new Rotation2d(Units.degreesToRadians(180)))
        );
        return trajectoryCommandFactory.createTrajectoryCommand(driveIn3Ball);
  }

}
