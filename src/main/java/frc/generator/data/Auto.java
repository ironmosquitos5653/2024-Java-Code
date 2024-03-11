package frc.generator.data;

import edu.wpi.first.math.geometry.Pose2d;

public class Auto {

    private String name;
    private double version;
    private Pose2d startingPose;
    private AutoCommand autoCommand;

    public Auto(String name, double version, Pose2d startingPose, AutoCommand autoCommand) {
        this.name = name;
        this.version = version;
        this.startingPose = startingPose;
        this.autoCommand = autoCommand;
    }
        
    public String getName() {
        return name;
    }

    public String getClassName() {
        return (name.substring(0,1) + name.substring(1)).replaceAll(" ", "");
    }

    public double getVersion() {
        return version;
    }

    public Pose2d getStartingPose() {
        return startingPose;
    }

    public AutoCommand getAutoCommand() {
        return autoCommand;
    }

    
}
