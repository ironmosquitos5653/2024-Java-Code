// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.time.LocalDateTime;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LifterSubsystem;

public class LiftCommand extends Command {

  private LifterSubsystem m_LifterSubsystem;
  private double m_angle;

  public LiftCommand(LifterSubsystem lifterSubsystem, double angle) {
    m_LifterSubsystem = lifterSubsystem;
    m_angle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putString("Liftezz", m_angle + " - " + LocalDateTime.now().toString());
    m_LifterSubsystem.setAngle(m_angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
