package frc.robot.commands.autos;

import java.util.Dictionary;
import java.util.Hashtable;

import edu.wpi.first.wpilibj2.command.Command;

public class CommandRegistry {

    static Dictionary<String, Command> commands = new Hashtable<String,Command>();

    public static void register(String name, Command command) {
        commands.put(name, command);
    }

    public static Command getCommand(String name) {
        return commands.get(name);
    }
    
}
