// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {

  private static final int deviceIDclimb = 19;
  private CANSparkMax climb;

  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {
    climb = new CANSparkMax(deviceIDclimb, MotorType.kBrushless);
    climb.restoreFactoryDefaults();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void up(double speed) {
    climb.set(-speed);
  }

  public void down(double speed) {
    climb.set(speed);
  }

  public void stop() {
    climb.set(0);
  }
}
