// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.LifterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AimLifterCommand extends PIDCommand {
  /** Creates a new AimLifterCommand. */
  public AimLifterCommand(LifterSubsystem lifterSubsystem) {
    super(
        // The controller that the command will use
        new PIDController(.01, 0, 0),
        // This should return the measurement
        () -> lifterSubsystem.getAngle(),
        // This should return the setpoint (can also be a constant)
        () -> 30,
        // This uses the output
        output -> {
          if(output > .2) {
            output = .2;
          }
          lifterSubsystem.up(output);
        });
    addRequirements(lifterSubsystem);
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
