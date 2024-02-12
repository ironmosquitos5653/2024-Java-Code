// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LifterSubsystem;

public class TestCommand extends Command {

  public LifterSubsystem m_LifterSubsystem;

  /** Creates a new TestCommand. */
  public TestCommand(LifterSubsystem lifterSubsystem) {
    m_LifterSubsystem = lifterSubsystem;
    addRequirements(lifterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_LifterSubsystem.up(.3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_LifterSubsystem.up(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
