// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private static final int deviceIDintakeAdvance = 10;
  private CANSparkMax intakeAdvance;

  private static final int deviceIDintake = 11;
  private CANSparkMax intake;

  private static final int deviceIDdecider = 12;
  private CANSparkMax decider;
  DigitalInput limit = new DigitalInput(4);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeAdvance = new CANSparkMax(deviceIDintakeAdvance, MotorType.kBrushless);
    intakeAdvance.restoreFactoryDefaults();
    intake = new CANSparkMax(deviceIDintake, MotorType.kBrushless);
    intake.restoreFactoryDefaults();
    decider = new CANSparkMax(deviceIDdecider, MotorType.kBrushless);
    decider.restoreFactoryDefaults();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("switch", limit.get());
  }

  public void intakeOn(double speed) {
    intake.set(-speed);
    intakeAdvance.set(speed);
    decider.set(speed);
  }

    public void ampUp(double speed) {
    intake.set(-speed);
    intakeAdvance.set(speed);
    decider.set(-speed);
  }

 public void advanceOn(double speed) {
    intakeAdvance.set(speed);
    decider.set(speed);
  }

 public void advanceOff(double speed) {
    intakeAdvance.set(0);
    decider.set(0);
  }

 public void ampAdvance(double speed) {
    intakeAdvance.set(speed);
    decider.set(-speed);
  }
  
  public void intakeOff() {
    intake.set(0);
    intakeAdvance.set(0);
    decider.set(0);
  }

  public boolean isLoaded() {
    return ! limit.get();
  }
}
