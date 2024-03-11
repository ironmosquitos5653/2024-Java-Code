package frc.generator.data;

 public class PathAutoCommand implements AutoCommand {
    private String pathName;
    private String functionName;

    public PathAutoCommand(String pathName) {
        this.pathName = pathName;
        this.functionName = "build" + pathName.substring(0,1).toUpperCase() + pathName.substring(1);
    }

    public String getPathName() {
        return pathName;
    }

    public String getFunctionName() {
        return functionName;
    }

    @Override
    public void generate(StringBuilder sb, int indent) {
        sb.append(String.format("%s(driveSubsystem, trajectoryCommandFactory)", functionName));
    }
}