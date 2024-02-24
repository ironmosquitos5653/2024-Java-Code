// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.fasterxml.jackson.core.sym.Name;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.LiftCommand;
import frc.robot.commands.ShootAutoCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.ShooterToggleCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LifterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/** Add your docs here. */
public class AutonomousManager {

 private IntakeSubsystem m_IntakeSubsystem;
 private LifterSubsystem m_LifterSubsystem;
 private ShooterSubsystem m_ShooterSubsystem;

  public AutonomousManager(IntakeSubsystem intakeSubsystem, LifterSubsystem lifterSubsystem, ShooterSubsystem shooterSubsystem) {
        m_IntakeSubsystem = intakeSubsystem;
        m_LifterSubsystem = lifterSubsystem;
        m_ShooterSubsystem = shooterSubsystem;
  } 

public void initialize() {
    NamedCommands.registerCommand("IntakeOn", new IntakeCommand(m_IntakeSubsystem));
    NamedCommands.registerCommand("Shoot", new ShootAutoCommand(m_ShooterSubsystem, m_IntakeSubsystem, .5));
    // NamedCommands.registerCommand("Lift 30", new RunCommand(() -> m_LifterSubsystem.setAngle(30), m_LifterSubsystem));
    NamedCommands.registerCommand("ShooterOn", new ShooterToggleCommand(m_ShooterSubsystem, .5));
    NamedCommands.registerCommand("ShooterOff", new ShooterToggleCommand(m_ShooterSubsystem, 0));
    NamedCommands.registerCommand("Lift 40", new LiftCommand(m_LifterSubsystem, 40));
    NamedCommands.registerCommand("Lift 35", new LiftCommand(m_LifterSubsystem, 36));
    NamedCommands.registerCommand("Lift 53", new LiftCommand(m_LifterSubsystem, 53));
    NamedCommands.registerCommand("Lift B", new LiftCommand(m_LifterSubsystem, 36));
    NamedCommands.registerCommand("Lift Stage Side", new LiftCommand(m_LifterSubsystem, 32));
  }
}
