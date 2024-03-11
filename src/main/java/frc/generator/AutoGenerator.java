package frc.generator;

import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;

import javax.swing.RepaintManager;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import frc.generator.data.Auto;
import frc.generator.data.AutoCommand;
import frc.generator.data.AutoPath;
import frc.generator.data.ParallelAutoCommand;
import frc.generator.data.PathAutoCommand;
import frc.generator.data.SequentialAutoCommand;
import frc.generator.data.Waypoint;

public class AutoGenerator {

    public static String PATHPLANNER_PATH = "src/main/deploy/pathplanner";
    public static String AUTO_GEN_PATH = "src/main/java/frc/robot/commands/autos";
    
    private static AprilTagFieldLayout layout;

    public AutoGenerator() {

    }

    private static String ClassTemplate = """
package frc.robot.commands.autos;
import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.TrajectoryCommandFactory;
import frc.robot.subsystems.DriveSubsystem;
import java.util.List;

public class $AUTO_NAME {
  public static Pose2d StartPose = $START_POSE;

  public static Command buildAuto(DriveSubsystem driveSubsystem, TrajectoryCommandFactory trajectoryCommandFactory) {
    return $COMMAND_LIST;
  } 

  $PATH_FUNCTIONS
}
""";

    public void generateAuto(Auto auto) throws Exception {
        String classText = ClassTemplate.replace("$AUTO_NAME", auto.getClassName())
            .replace("$START_POSE", buildStartPose(auto.getStartingPose()))
            .replace("$COMMAND_LIST", buildCommandList(auto))
            .replace("$PATH_FUNCTIONS", generatePaths(auto));

        File output = new File(AUTO_GEN_PATH + "/" + auto.getClassName() + ".java");
        if (output.exists()) {
            output.delete();
        }
        FileWriter io = new FileWriter(output);
        io.write(classText);
        io.close();
    }

    public void generateRedAuto(Auto auto) throws Exception {
        auto.flipToRed();
        generateAuto(auto);
    }

    public String generatePaths(Auto auto) throws Exception {
        StringBuilder sb = new StringBuilder();
        generatePaths(auto.getAutoCommand(), sb);
        return sb.toString();
    }

    public void generatePaths(AutoCommand command, StringBuilder sb) throws Exception {
        if (command instanceof PathAutoCommand) {
            generatePath((PathAutoCommand) command, sb);
        } else if (command instanceof SequentialAutoCommand) {
            generatePaths(((SequentialAutoCommand)command).getCommands(), sb);
        } else if (command instanceof ParallelAutoCommand) {
            generatePaths(((ParallelAutoCommand)command).getCommands(), sb);
        }
    }

    public void generatePaths(AutoCommand[] commands, StringBuilder sb) throws Exception {
        for (AutoCommand command : commands) {
            generatePaths(command, sb);
        }
    }

    private static String BUILD_PATH_TEMPLATE = """

  public static Command $FUNCTION_NAME(DriveSubsystem driveSubsystem, TrajectoryCommandFactory trajectoryCommandFactory) {
    Trajectory trajectory = trajectoryCommandFactory.createTrajectory(
        new Pose2d($START_X, $START_Y,  new Rotation2d(Units.degreesToRadians($START_DEGREES))),
        $INTERIOR_WAYPOINTS,
        new Pose2d($END_X, $END_Y, new Rotation2d(Units.degreesToRadians($END_DEGREES)))
    );
    return trajectoryCommandFactory.createTrajectoryCommand(trajectory);
  }
            """;

    private void generatePath(PathAutoCommand command, StringBuilder sb) throws Exception {
        AutoPath path = command.getAutoPath();
        Waypoint start = path.getStartPoint();
        Waypoint[] interior = path.getInteriorWaypoints();
        Waypoint end = path.getEndPoint();
        String sx = "" + start.getAnchor().getX();
        String sy = "" + start.getAnchor().getY();
        String sd = "" + path.getStartAngle();
        String ex = "" + end.getAnchor().getX();
        String ey = "" + end.getAnchor().getY();
        String ed = "" + path.getEndState().getRotation();

        String interiorWaypoints = generateInteriorWaypoints(interior);

        String function =
            BUILD_PATH_TEMPLATE
                .replace("$FUNCTION_NAME", command.getFunctionName())
                .replace("$START_X", ex)
                .replace("$START_Y", ey)
                .replace("$START_DEGREES", sd)
                .replace("$INTERIOR_WAYPOINTS", interiorWaypoints)
                .replace("$END_X", ex)
                .replace("$END_Y", ey)
                .replace("$END_DEGREES", ed);

        sb.append(function);

        
    }

    private String generateInteriorWaypoints(Waypoint[] waypoints) {
        if (waypoints.length == 0 ) {
            return "new ArrayList<Translation2d>()";
        }
        StringBuilder sb = new StringBuilder();
        sb.append("List.of(");
        sb.append(System.lineSeparator());
        for (int i = 0; i < waypoints.length; i++) {
            double x = waypoints[i].getAnchor().getX();
            double y = waypoints[i].getAnchor().getY();
            sb.append("        new Translation2d(" + x + ", " + y + ")");
            if (i != waypoints.length -1) {
                sb.append(",");
            }
            sb.append(System.lineSeparator());
        }
        sb.append("      )");
        return sb.toString();
    }

    public String buildStartPose(Pose2d pose) {
        return String.format("new Pose2d(%f, %f, new Rotation2d(Units.degreesToRadians(%f)))", pose.getX(), pose.getY(), pose.getRotation().getDegrees());
    }

    public String buildCommandList(Auto auto) {
        StringBuilder sb = new StringBuilder();

        auto.getAutoCommand().generate(sb, 6);
        return sb.toString();
    }

    public static void indent(StringBuilder sb, int indent) {
        for(int i = 0; i< indent; i++) {
            sb.append(" ");
        }
    }
     
    public static AprilTagFieldLayout getAprilTagFieldLayout() {
        if (layout == null) {
            try {
                layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
            } catch (Exception ex) {
                System.err.println(ex.getMessage());
                System.err.println(ex.getStackTrace());
            }
        }
        return layout;
    }

    public static void main(String[] args) throws Exception {
        ArrayList<Auto> autos = new ArrayList<Auto>();
        AutoGenerator ag = new AutoGenerator();

        for (File auto : new File(PATHPLANNER_PATH + "/autos").listFiles()) {
            autos.add(AutoParser.parseAuto(auto));
        }

        for(Auto a : autos) {
            ag.generateAuto(a);
            if (a.getName().contains("Blue")) {
                ag.generateRedAuto(a);
            }
        }
    }
}
