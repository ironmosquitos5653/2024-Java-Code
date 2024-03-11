// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what
 * you are doing, do not modify this file except to change the parameter class to the startRobot
 * call.
 */
public final class Main {
  private Main() {}

  /**
   * Main initialization function. Do not perform any initialization here.
   *
   * <p>If you change your main robot class, change the parameter type.
   */
  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);

        //Transform3d trans = new Transform3d(new Translation3d(-0.3425, 0.0, -0.233), new Rotation3d(0,0,180));
        //Pose3d thePose = new Pose3d(0,0,0, new Rotation3d(0,0,180));

        //Pose3d x = thePose.transformBy(trans);
        //System.out.println(x);
  }
}
