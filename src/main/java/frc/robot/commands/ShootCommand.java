// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LifterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command {
  public DriveSubsystem m_DriveSubsystem;
  public ShooterSubsystem m_ShooterSubsystem;
  public IntakeSubsystem m_IntakeSubsystem;
  public LifterSubsystem m_LifterSubsystem;
  public double m_speed;
  Timer timer = new Timer();
  
  /** Creates a new ShootCommand. */
  public ShootCommand(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem, LifterSubsystem lifterSubsystem, DriveSubsystem driveSubsystem, double speed) {
    m_DriveSubsystem = driveSubsystem;
    m_ShooterSubsystem = shooterSubsystem;
    m_IntakeSubsystem = intakeSubsystem;
    m_LifterSubsystem = lifterSubsystem;
    m_speed = speed;
    addRequirements(m_ShooterSubsystem, m_IntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_DriveSubsystem.setAutoAim(true);
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ShooterSubsystem.shoot(.5);
    if(timer.hasElapsed(.8)) {
      m_ShooterSubsystem.shootAdvance(1);
      m_IntakeSubsystem.advanceOn(1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveSubsystem.setAutoAim(false);
    m_ShooterSubsystem.shootOff(0);
    m_IntakeSubsystem.intakeOff(); 
    m_LifterSubsystem.setAngle(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(1.5);
  }
}
