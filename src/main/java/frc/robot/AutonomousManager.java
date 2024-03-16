// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.LiftCommand;
import frc.robot.commands.ShootAutoCommand;
import frc.robot.commands.ShooterToggleCommand;
import frc.robot.commands.autos.CommandRegistry;
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

        CommandRegistry.m_IntakeSubsystem = intakeSubsystem;
        CommandRegistry.m_LifterSubsystem = lifterSubsystem;
        CommandRegistry.m_ShooterSubsystem = shooterSubsystem;
  } 

public void initialize() {
    register("IntakeOn", new IntakeCommand(m_IntakeSubsystem));
    register("Shoot", new ShootAutoCommand(m_ShooterSubsystem, m_IntakeSubsystem, .5));
    // NamedCommands.registerCommand("Lift 30", new RunCommand(() -> m_LifterSubsystem.setAngle(30), m_LifterSubsystem));
    register("ShooterOn", new ShooterToggleCommand(m_ShooterSubsystem, .5));
    register("ShooterOff", new ShooterToggleCommand(m_ShooterSubsystem, 0));
    register("Lift 40", new LiftCommand(m_LifterSubsystem, 40));
    register("Lift 35", new LiftCommand(m_LifterSubsystem, 36));
    register("Lift 53", new LiftCommand(m_LifterSubsystem, 53));
    register("Lift B", new LiftCommand(m_LifterSubsystem, 36));
    register("Lift Stage Side", new LiftCommand(m_LifterSubsystem, 32));
  }

  private void register(String name, Command command) {
        NamedCommands.registerCommand(name, command);
        CommandRegistry.register(name, command);
  }
}
