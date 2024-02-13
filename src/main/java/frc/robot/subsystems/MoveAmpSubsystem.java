// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MoveAmpSubsystem extends SubsystemBase {

  private static final int deviceIDamp = 17;
  private CANSparkMax amp;

  private AbsoluteEncoder ampEncoder;

  private double targetPostiton = .56;

  PIDController pidController;

  public MoveAmpSubsystem() {
    pidController = new PIDController(1.2, 0.1, 0);

    pidController.setSetpoint(.56);
    amp = new CANSparkMax(deviceIDamp, MotorType.kBrushless);
    amp.restoreFactoryDefaults();
    ampEncoder = amp.getAbsoluteEncoder(Type.kDutyCycle);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("position", getMeasurement());
    SmartDashboard.putNumber("Set Point", targetPostiton);
    double speed = pidController.calculate(getMeasurement());
    SmartDashboard.putNumber("Speed", speed);
    if (speed > .2) {
      speed = .2;
    }
    amp.set(speed);
  }
  
  public double getMeasurement() {
    return ampEncoder.getPosition();
  }

  public void setTargetPosition (double targetPostiton){
    this.targetPostiton = targetPostiton;
    pidController.setSetpoint(targetPostiton);
  }

  public double getTargetPosition (){
    return targetPostiton;
  }

}
