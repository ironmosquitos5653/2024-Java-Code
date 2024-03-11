package frc.generator.data;

import frc.generator.PathParser;

public class PathAutoCommand implements AutoCommand {
    private String pathName;
    private String functionName;
    private AutoPath autoPath;

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
    
    public void flitToRed() {

    }
    
    public AutoPath getAutoPath() {
        if (autoPath == null) {
            try {
                autoPath = PathParser.parsePath(getPathName());
            } catch (Exception ex) {
                System.out.println(ex.getMessage());
                System.out.println(ex.getStackTrace());
            }
        }
        return autoPath;
    }

    public void flipToRed() {
        getAutoPath().flipToRed();
    }


    @Override
    public void generate(StringBuilder sb, int indent) {
        sb.append(String.format("%s(driveSubsystem, trajectoryCommandFactory)", functionName));
    }
}