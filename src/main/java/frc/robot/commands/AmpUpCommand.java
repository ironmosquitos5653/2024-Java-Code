// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MoveAmpSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AmpUpCommand extends Command {
 
  IntakeSubsystem m_IntakeSubsystem;
  MoveAmpSubsystem m_MoveAmpSubsystem;
  Timer timer = new Timer();

  public AmpUpCommand(IntakeSubsystem intakeSubsystem, MoveAmpSubsystem moveAmpSubsystem) {
    m_IntakeSubsystem = intakeSubsystem;
    m_MoveAmpSubsystem = moveAmpSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_IntakeSubsystem.ampUp(.5);
    m_MoveAmpSubsystem.setTargetPosition(.15);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.intakeOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(1);
  }
}
