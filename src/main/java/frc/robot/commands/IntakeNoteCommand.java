// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeNoteCommand extends Command {
  /** Creates a new SuckNote. */
private AmpSubsystem m_AmpSubsystem;
private IntakeSubsystem m_IntakeSubsystem;
private Timer timer;
  public IntakeNoteCommand(AmpSubsystem ampSubsystem, IntakeSubsystem intakeSubsystem) {
    m_AmpSubsystem = ampSubsystem;
    m_IntakeSubsystem = intakeSubsystem;
    addRequirements(ampSubsystem);
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = new Timer();
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_IntakeSubsystem.ampAdvance(.5);
    m_AmpSubsystem.setSpeed(.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.ampAdvance(0);
    m_AmpSubsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ! m_IntakeSubsystem.isLoaded();
  }
}
