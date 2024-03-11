// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;

public class LifterSubsystem extends SubsystemBase {

  private static final int deviceIDlifter = 16;
  private CANSparkMax lifter; 

  private PIDController pidController;

  private AbsoluteEncoder encoder = null;
  private double targetAngle = 0;

  /** Creates a new LifterSubsystem. */
  public LifterSubsystem(PoseEstimatorSubsystem poseEstimatorSubsystem) {
    lifter = new CANSparkMax(deviceIDlifter, MotorType.kBrushless);
    lifter.restoreFactoryDefaults();
    encoder = lifter.getAbsoluteEncoder(Type.kDutyCycle);
    pidController = new PIDController(.015, .0003, 0);

    ShuffleboardTab tab = Shuffleboard.getTab("Lifter");
    tab.addNumber("Error", pidController::getPositionError).withPosition(0, 0).withSize(2, 0);
    
    tab.addNumber("SetPoint", pidController::getSetpoint).withPosition(1, 0).withSize(2, 0);
    tab.addNumber("Speed", lifter::get).withPosition(2, 0).withSize(2, 0);
    tab.addNumber("Angle", this::getAngle).withPosition(3, 0).withSize(2, 0);
    tab.addNumber("Target Angle", poseEstimatorSubsystem::getVerticalShootAngle).withPosition(4, 0).withSize(2, 0);
    tab.addNumber("AbsoluteEncoder", encoder::getPosition).withPosition(5, 0).withSize(2,0);
    tab.addNumber("EncoderAngle", this::getEncoderAngle).withPosition(6, 0).withSize(2,0);
  }


  @Override
  public void periodic() {
    isDisabled();
    //m_PoseEstimatorSubsystem.getVerticalShootAngle();
    // 53 at speaker
    // 40 at stage
    // 31 speed .5
    if (targetAngle > 16) {
      pidController.setSetpoint(targetAngle);
      double speed = pidController.calculate(getAngle());
      if(speed > .2) {
        speed = .2;
      } else if ( speed < -.2) {
        speed = -.2;
      }
 
      lifter.set(-speed);
    } else {
      lifter.set(0);
    }
  }

  private boolean disabled = true;

  private boolean isDisabled() {
    if (disabled != RobotState.isDisabled()) {
      if ( ! RobotState.isDisabled()) {
        pidController.reset();
      }
    }
    disabled = ! RobotState.isDisabled();

    return disabled;
  }

  public void up(double speed) {
    lifter.set(-speed);
  }

  private double getEncoderPosition() {
    double value = encoder.getPosition();
    if (value > .8) {
      value = 1 - value;
    }
    return value;
  }

  private double getEncoderAngle() {
    return  (getEncoderPosition() * 360) + 15.2;
  }

  public double getAngle() {
    return getEncoderAngle();
    //return m_gyro.getPitch().getValue() + 90;
  }

  public void setAngle(double angle) {
    targetAngle = angle;
  }

  public double getTargetAngle() {
    return targetAngle;
  }

}
