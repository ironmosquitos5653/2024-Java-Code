package frc.robot.commands.autos;

import java.util.Dictionary;
import java.util.Hashtable;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.LiftCommand;
import frc.robot.commands.ShootAutoCommand;
import frc.robot.commands.ShooterToggleCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LifterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class CommandRegistry {

    public static IntakeSubsystem m_IntakeSubsystem;
    public static LifterSubsystem m_LifterSubsystem;
    public static ShooterSubsystem m_ShooterSubsystem;

    static Dictionary<String, Command> commands = new Hashtable<String,Command>();

    public static void register(String name, Command command) {
        commands.put(name, command);
    }

    public static Command getCommand(String name) {
        if (name == "IntakeOn") return new IntakeCommand(m_IntakeSubsystem);
    if (name == "Shoot") return new ShootAutoCommand(m_ShooterSubsystem, m_IntakeSubsystem, .5);
    // NamedCommands.registerCommand("Lift 30", new RunCommand(() -> m_LifterSubsystem.setAngle(30), m_LifterSubsystem));
    if (name == "ShooterOn") return new ShooterToggleCommand(m_ShooterSubsystem, .5);
    if (name == "ShooterOff") return new ShooterToggleCommand(m_ShooterSubsystem, 0);
    if (name == "Lift 40") return new LiftCommand(m_LifterSubsystem, 40);
    if (name == "Lift GA") return new LiftCommand(m_LifterSubsystem, 34);
    if (name == "Lift PP") return new LiftCommand(m_LifterSubsystem, 30);
    if (name == "Lift 53") return new LiftCommand(m_LifterSubsystem, 54);
    if (name == "Lift B") return new LiftCommand(m_LifterSubsystem, 37);
    if (name == "Lift C") return new LiftCommand(m_LifterSubsystem, 35);
    if (name == "Lift SR") return new LiftCommand(m_LifterSubsystem, 35);
    if (name == "Lift BM2") return new LiftCommand(m_LifterSubsystem, 50);
    if (name == "Lift BM3") return new LiftCommand(m_LifterSubsystem, 39);
    if (name == "Lift Shoot") return new LiftCommand(m_LifterSubsystem, 41);
    if (name == "Lift RS") return new LiftCommand(m_LifterSubsystem, 32);
    if (name == "Lift BS") return new LiftCommand(m_LifterSubsystem, 32);
    if (name == "Lift Stage Side") return new LiftCommand(m_LifterSubsystem, 32);
        return null;
    }
    
}
