        package frc.robot.commands.autos;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.TrajectoryCommandFactory;
import frc.robot.subsystems.DriveSubsystem;

public class AutoUtil {

    public static void addAutos(SendableChooser<String> chooser) {
        chooser.addOption("Back and forth", "Back and forth");
        chooser.addOption("RedRightAuto", "RedRightAuto");
        chooser.addOption("RedMiddle", "RedMiddle");
        chooser.addOption("RedLeft", "RedLeft");
        chooser.addOption("DriveBackRedRight", "DriveBackRedRight");
        chooser.addOption("DriveBackNegative", "DriveBackNegative");
        chooser.addOption("Test", "Test");
        chooser.addOption("BlueRightAuto", "BlueRightAuto");
        chooser.addOption("BlueMiddle", "BlueMiddle");
        chooser.addOption("BlueLeft", "BlueLeft");


        // PathPlanner autos
        chooser.addOption("PP Blue Left", "PP Blue Left");
        chooser.addOption("PP Blue Middle", "PP Blue Middle");
        chooser.addOption("PP Blue Right", "PP Blue Right");
        chooser.addOption("PP Red Left", "PP Blue Right");
        chooser.addOption("PP Red Middle", "PP Blue Middle");
        chooser.addOption("PP Red Right", "PP Blue Left");
    }

    public static Command getAuto(String name, DriveSubsystem driveSubsystem, TrajectoryCommandFactory trajectoryCommandFactory) {
        switch(name) {
            case "Back and forth": return Test1Blue.buildAuto(driveSubsystem, trajectoryCommandFactory);
            case "RedRightAuto": return RedRightAuto.buildAuto(driveSubsystem, trajectoryCommandFactory);
            case "RedMiddle": return RedMiddle.buildAuto(driveSubsystem, trajectoryCommandFactory);
            case "RedLeft": return RedLeft.buildAuto(driveSubsystem, trajectoryCommandFactory);
            case "DriveBackRedRight": return Test1Blue.buildAuto(driveSubsystem, trajectoryCommandFactory);
            case "DriveBackNegative": return Test1Blue.buildAuto(driveSubsystem, trajectoryCommandFactory);
            case "Test": return Test1Blue.buildAuto(driveSubsystem, trajectoryCommandFactory);
            case "BlueRightAuto": return BlueRight.buildAuto(driveSubsystem, trajectoryCommandFactory);
            case "BlueMiddle": return BlueMiddle.buildAuto(driveSubsystem, trajectoryCommandFactory);
            case "BlueLeft": return BlueLeftAuto.buildAuto(driveSubsystem, trajectoryCommandFactory);

            case "PP Blue Right": return AutoBuilder.buildAuto("BlueRight");
            case "PP Blue Middle": return AutoBuilder.buildAuto("BlueMiddle");
            case "PP Blue Left": return AutoBuilder.buildAuto("BlueLeftAuto");
        }
        return null;
    }
}
