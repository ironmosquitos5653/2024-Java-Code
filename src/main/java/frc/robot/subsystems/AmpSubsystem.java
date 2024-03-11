// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AmpSubsystem extends SubsystemBase {


  private static final int deviceIDwheels = 18;
  private CANSparkMax wheels;

  /** Creates a new AmpSubsystem. */
  public AmpSubsystem() {
    wheels = new CANSparkMax(deviceIDwheels, MotorType.kBrushless);
    wheels.restoreFactoryDefaults();
  }

  @Override
  public void periodic() {

  }

  public void setSpeed(double speed) {
    wheels.set(-speed);
  }

  public void ampShoot(double speed) {
    wheels.set(speed);
  }

}
