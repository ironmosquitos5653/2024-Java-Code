package frc.generator.data;

import frc.generator.AutoGenerator;

public class ParallelAutoCommand implements AutoCommand {
    private AutoCommand[] commands;

    public ParallelAutoCommand(AutoCommand[] commands) {
        this.commands = commands;
    }

    public AutoCommand[] getCommands() {
        return commands;
    }

    @Override
    public void generate(StringBuilder sb, int indent) {
        boolean first = true;
        for (AutoCommand command : getCommands()) {
            sb.append(System.lineSeparator());
            if( ! first ) {
                AutoGenerator.indent(sb, indent);
                sb.append(".alongWith(");
            } else {
                AutoGenerator.indent(sb, indent);
            }
            
            command.generate(sb, indent + 2);

            first = false;
        }
        sb.append(")");
    }

}