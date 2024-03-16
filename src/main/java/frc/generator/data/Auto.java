package frc.generator.data;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.generator.AutoGenerator;

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
        updateAngles();
    }
        
    public String getName() {
        return name;
    }

    public void flipToRed() {
        double x = AutoGenerator.getAprilTagFieldLayout().getFieldLength() - startingPose.getX();
        startingPose = new Pose2d(new Translation2d(x, startingPose.getY()), new Rotation2d(Units.degreesToRadians(Waypoint.flipAngle(startingPose.getRotation().getDegrees()))));
        name = name.replace("Blue", "Red");
        if (name.contains("Left")) {
            name = name.replace("Left", "Right");
        } else if (name.contains("Right")) {
            name = name.replace("Right", "Left");
        }
        flipToRed(autoCommand);
    }
    
    public void flipToRed(AutoCommand command) {
        if (command instanceof PathAutoCommand) {
            ((PathAutoCommand)command).flipToRed();
        } else if (command instanceof SequentialAutoCommand) {
            for (AutoCommand c : ((SequentialAutoCommand)command).getCommands()) {
                flipToRed(c);
            }
        } else if (command instanceof ParallelAutoCommand) {
            for (AutoCommand c : ((ParallelAutoCommand)command).getCommands()) {
                flipToRed(c);
            }
        }
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

    // Start angles are not configured in pathplanner markup.  copying end to the start of the next one.
    private void updateAngles() {
        updateAngles(getAutoCommand(), startingPose.getRotation().getDegrees() );
    }

    private double updateAngles(AutoCommand command, double previousAngle)
    {
        if (command instanceof PathAutoCommand) {
            PathAutoCommand p = (PathAutoCommand) command;
            p.getAutoPath().setStartAngle(previousAngle);
            previousAngle = p.getAutoPath().getEndState().getRotation();
            
        } else if (command instanceof SequentialAutoCommand) {
            for (AutoCommand c : ((SequentialAutoCommand)command).getCommands()) {
                previousAngle = updateAngles(c, previousAngle);
            }
        } else if (command instanceof ParallelAutoCommand) {
            for (AutoCommand c : ((ParallelAutoCommand)command).getCommands()) {
                previousAngle = updateAngles(c, previousAngle);
            }
        }
        return previousAngle;
    }
}
