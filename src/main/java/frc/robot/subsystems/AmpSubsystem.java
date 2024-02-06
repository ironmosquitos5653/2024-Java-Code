// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AmpSubsystem extends SubsystemBase {

  private static final int deviceIDamp = 17;
  private CANSparkMax amp;

  /** Creates a new AmpSubsystem. */
  public AmpSubsystem() {
    amp = new CANSparkMax(deviceIDamp, MotorType.kBrushless);
    amp.restoreFactoryDefaults();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void up(double speed) {
  amp.set(speed);
  }

  public void stop(double speed) {
  amp.set(0);
  }
}
