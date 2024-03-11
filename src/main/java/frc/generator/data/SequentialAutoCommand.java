package frc.generator.data;

import frc.generator.AutoGenerator;

public class SequentialAutoCommand implements AutoCommand {

    private AutoCommand[] commands;

    public SequentialAutoCommand(AutoCommand[] commands) {
        this.commands = commands;
    }

    public AutoCommand[] getCommands() {
        return commands;
    }

    @Override
    public void generate(StringBuilder sb, int indent) {
        for (int i = 0 ;i < commands.length; i ++) {
            if( i != 0 ) {
                AutoGenerator.indent(sb, indent);
                sb.append(".andThen(");
            }

            commands[i].generate(sb, indent + 2);

            if ( i != 0 ) {
                sb.append(")");
            }

            if (i != commands.length - 1) {
                sb.append(System.lineSeparator());
            }
        }
    }


}