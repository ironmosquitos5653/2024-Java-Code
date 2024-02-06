// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class LifterSubsystem extends SubsystemBase {

  private static final int deviceIDlifter = 16;
  private CANSparkMax lifter; 
  private final Pigeon2 m_gyro = new Pigeon2(21);
  private double angleOffset;
  /** Creates a new LifterSubsystem. */
  public LifterSubsystem() {
    lifter = new CANSparkMax(deviceIDlifter, MotorType.kBrushless);
    lifter.restoreFactoryDefaults();
    m_gyro.reset();
    angleOffset = m_gyro.getPitch().getValue();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("pitch", m_gyro.getPitch().getValue());
    SmartDashboard.putNumber("roll", m_gyro.getRoll().getValue());
    SmartDashboard.putNumber("yaw", m_gyro.getYaw().getValue());
  }

  public void up(double speed) {
    lifter.set(-speed);
  }

    public void stop(double speed) {
    lifter.set(0);
  }

  public double getAngle() {
    return m_gyro.getPitch().getValue() - angleOffset;
  }
}
