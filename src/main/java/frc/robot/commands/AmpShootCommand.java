// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpSubsystem;
import frc.robot.subsystems.MoveAmpSubsystem;

public class AmpShootCommand extends Command {
  private AmpSubsystem m_AmpSubsystem;
  private MoveAmpSubsystem m_MoveAmpSubsystem;
  private Timer timer;

  public AmpShootCommand(AmpSubsystem ampSubsystem, MoveAmpSubsystem moveAmpSubsystem) {
    m_AmpSubsystem = ampSubsystem;
    m_MoveAmpSubsystem = moveAmpSubsystem;
    addRequirements(ampSubsystem);
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
    m_AmpSubsystem.ampShoot(.6);
    if (timer.hasElapsed(.25)) {
      m_MoveAmpSubsystem.setTargetPosition(.35);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_AmpSubsystem.ampShoot(0);
    m_MoveAmpSubsystem.setTargetPosition(.55);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(1.5);
  }
}
