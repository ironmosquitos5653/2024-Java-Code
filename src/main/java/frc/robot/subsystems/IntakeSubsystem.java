// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private static final int deviceIDintakeAdvance = 10;
  private CANSparkMax intakeAdvance;

  private static final int deviceIDintake = 11;
  private CANSparkMax intake;

  private static final int deviceIDdecider = 12;
  private CANSparkMax decider;

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
    // This method will be called once per scheduler run
  }

  public void intakeOn(double speed) {
    intake.set(speed);
    intakeAdvance.set(speed);
  }

  public void intakeOff(double speed) {
    intake.set(0);
    intakeAdvance.set(0);
  }
}
