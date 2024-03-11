// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MoveAmpSubsystem extends SubsystemBase {

  private static final int deviceIDamp = 17;
  private CANSparkMax amp;

  private AbsoluteEncoder ampEncoder;

  private double targetPostiton = .55;

  PIDController pidController;

  public MoveAmpSubsystem() {
    pidController = new PIDController(1.2, 0.1, 0);

    pidController.setSetpoint(.55);
    amp = new CANSparkMax(deviceIDamp, MotorType.kBrushless);
    amp.restoreFactoryDefaults();
    ampEncoder = amp.getAbsoluteEncoder(Type.kDutyCycle);
    
    ShuffleboardTab tab = Shuffleboard.getTab("Amp");
    tab.addNumber("Error", pidController::getPositionError).withPosition(0, 0).withSize(2, 0);
    tab.addNumber("position", this::getMeasurement).withPosition(1, 0).withSize(2, 0);
    tab.addNumber("setpoint", pidController::getSetpoint).withPosition(2, 0).withSize(2, 0);
    tab.addNumber("speed", amp::get).withPosition(3, 0).withSize(2, 0);
  }

  boolean off = false;
  @Override
  public void periodic() {
    isDisabled();
    if ( (targetPostiton == .55 && getMeasurement() > .55) || (off && targetPostiton == .55)) {
     amp.set(0);
     off = true;
    } else {
      off = false;
      double speed = pidController.calculate(getMeasurement());

      if(speed > .2) {
        speed = .2;
      }
      if (getMeasurement() < .3) {
        if (speed < -.1) {
          speed = -.1;
        }
      }
      amp.set(speed);
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
  
  public double getMeasurement() {
    return ampEncoder.getPosition();
  }

  public void setTargetPosition (double targetPostiton){
    this.targetPostiton = targetPostiton;
    pidController.setSetpoint(targetPostiton);
  }

  public double getTargetPosition() {
    return targetPostiton;
  }

}
