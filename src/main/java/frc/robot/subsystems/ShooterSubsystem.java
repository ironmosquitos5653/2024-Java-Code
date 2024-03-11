// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  private static final int deviceIDshootAdv = 13;
  private CANSparkMax shootAdvance;

  private static final int deviceIDshoot1 = 14;
  private CANSparkFlex shoot1;
  private static final int deviceIDshoot2 = 15;
  private CANSparkFlex shoot2;

  public ShooterSubsystem() {
    shootAdvance = new CANSparkMax(deviceIDshootAdv, MotorType.kBrushless);
    shootAdvance.restoreFactoryDefaults();
    shoot1 = new CANSparkFlex(deviceIDshoot1, MotorType.kBrushless);
    shoot1.restoreFactoryDefaults();
    shoot2 = new CANSparkFlex(deviceIDshoot2, MotorType.kBrushless);
    shoot2.restoreFactoryDefaults();
    //shootAngle = new CANSparkMax(deviceIDshootAngle, MotorType.kBrushless);
    //shootAngle.restoreFactoryDefaults();
  }

   /** Creates a new ShooterSubsystem. */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  
    SmartDashboard.putNumber("Shoot1 Velocity",  shoot1.getEncoder().getVelocity());
    SmartDashboard.putNumber("Shoot2 Velocity",  shoot2.getEncoder().getVelocity());
  }

  public void shoot(double speed) {
    shoot1.set(speed);
    shoot2.set(speed);
  }

  public void shootAdvance(double speed) {
    shootAdvance.set(speed);
  }

  public void shootOff(double speed) {
    shootAdvance.set(0);
    shoot1.set(0);
    shoot2.set(0);
  }

}
