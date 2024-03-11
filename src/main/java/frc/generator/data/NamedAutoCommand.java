package frc.generator.data;

public class NamedAutoCommand implements AutoCommand {
    private String name;

    public NamedAutoCommand(String name) {
        this.name = name;
    }

    public String getName() {
        return name;
    }

    @Override
    public void generate(StringBuilder sb, int indent) {
        sb.append(String.format("CommandRegistry.getCommand(\"%s\")", name));
    }
}