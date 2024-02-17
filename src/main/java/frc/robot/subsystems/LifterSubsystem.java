// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
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

  private PIDController pidController;
  private PoseEstimatorSubsystem m_PoseEstimatorSubsystem;

  /** Creates a new LifterSubsystem. */
  public LifterSubsystem(PoseEstimatorSubsystem poseEstimatorSubsystem) {
    m_PoseEstimatorSubsystem = poseEstimatorSubsystem;
    lifter = new CANSparkMax(deviceIDlifter, MotorType.kBrushless);
    lifter.restoreFactoryDefaults();
    m_gyro.reset();
    angleOffset = m_gyro.getPitch().getValue();

    pidController = new PIDController(.015, .0003, 0);

  }

  @Override
  public void periodic() {
    double targetAngle = m_PoseEstimatorSubsystem.getVerticalShootAngle();
    if (targetAngle > 0) {
      pidController.setSetpoint(targetAngle);
      SmartDashboard.putNumber("shooter setpoint", pidController.getSetpoint());
      SmartDashboard.putNumber("pitch", m_gyro.getPitch().getValue());
      SmartDashboard.putNumber("shootangle", getAngle());
      SmartDashboard.putNumber("error", pidController.getPositionError());
      double speed = pidController.calculate(getAngle());
      SmartDashboard.putNumber("speed", speed);
      if(speed > .2) {
        speed = .2;
      }
      lifter.set(-speed);
    } else {
      lifter.set(0);
    }
  }

  public void up(double speed) {
    lifter.set(-speed);
  }

  public double getAngle() {
    return m_gyro.getPitch().getValue() + 90;
  }
}
