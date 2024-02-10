// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AmpSubsystem extends SubsystemBase {

  private static final int deviceIDamp = 17;
  private CANSparkMax amp;
  private static final int deviceIDwheels = 18;
  private CANSparkMax wheels;
  private double targetPostiton = .99;


  private AbsoluteEncoder ampEncoder;
  /** Creates a new AmpSubsystem. */
  public AmpSubsystem() {
    amp = new CANSparkMax(deviceIDamp, MotorType.kBrushless);
    amp.restoreFactoryDefaults();
    wheels = new CANSparkMax(deviceIDwheels, MotorType.kBrushless);
    wheels.restoreFactoryDefaults();
    ampEncoder = amp.getAbsoluteEncoder(Type.kDutyCycle);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("amp Position", getPosition());
    SmartDashboard.putNumber("target position", getTargetPosition());
  }

  public void setSpeed(double speed) {
    wheels.set(-speed);
  }
 public void setAmpSpeed(double speed) {
    amp.set(-speed);
  }
  public double getPosition(){
    return ampEncoder.getPosition();
  }

  public void ampShoot(double speed) {
    wheels.set(speed);
  }

  public void setTargetPosition (double targetPostiton){
    this.targetPostiton = targetPostiton;
  }

  public double getTargetPosition (){
    return targetPostiton;
  }

  public void up(double speed) {
  amp.set(speed);
  }

  public void stop(double speed) {
  amp.set(0);
  }
}
