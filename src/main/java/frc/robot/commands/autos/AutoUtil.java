        package frc.robot.commands.autos;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.TrajectoryCommandFactory;
import frc.robot.subsystems.DriveSubsystem;

public class AutoUtil {

    public static void addAutos(SendableChooser<String> chooser) {
        chooser.addOption("Backandforth", "Backandforth");
        chooser.addOption("RedRightAuto", "RedRightAuto");
        chooser.addOption("BlueLeftAuto", "BlueLeftAuto");
        chooser.addOption("RedMiddle", "RedMiddle");
        chooser.addOption("BlueMiddle", "BlueMiddle");
        chooser.addOption("RedLeft", "RedLeft");
        chooser.addOption("BlueRight", "BlueRight");
        chooser.addOption("DriveBackRedRight", "DriveBackRedRight");
        chooser.addOption("DriveBackBlueLeft", "DriveBackBlueLeft");
        chooser.addOption("DriveBackNegative", "DriveBackNegative");
        chooser.addOption("Test", "Test");


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
            case "Backandforth": 
                driveSubsystem.setAutoStart(Backandforth.StartPose); 
 return Backandforth.buildAuto(driveSubsystem, trajectoryCommandFactory);
            case "RedRightAuto": 
                driveSubsystem.setAutoStart(RedRightAuto.StartPose); 
 return RedRightAuto.buildAuto(driveSubsystem, trajectoryCommandFactory);
            case "BlueLeftAuto": 
                driveSubsystem.setAutoStart(BlueLeftAuto.StartPose); 
            return BlueLeftAuto.buildAuto(driveSubsystem, trajectoryCommandFactory);
            case "RedMiddle": 
                driveSubsystem.setAutoStart(RedMiddle.StartPose); 
 return RedMiddle.buildAuto(driveSubsystem, trajectoryCommandFactory);
            case "BlueMiddle": 
                driveSubsystem.setAutoStart(BlueMiddle.StartPose); 
            return BlueMiddle.buildAuto(driveSubsystem, trajectoryCommandFactory);
            case "RedLeft": 
                driveSubsystem.setAutoStart(RedLeft.StartPose); 
 return RedLeft.buildAuto(driveSubsystem, trajectoryCommandFactory);
            case "BlueRight": 
                driveSubsystem.setAutoStart(BlueRight.StartPose); 
            return BlueRight.buildAuto(driveSubsystem, trajectoryCommandFactory);
            case "DriveBackRedRight": 
                driveSubsystem.setAutoStart(DriveBackRedRight.StartPose); 
 return DriveBackRedRight.buildAuto(driveSubsystem, trajectoryCommandFactory);
            case "DriveBackBlueLeft": 
                driveSubsystem.setAutoStart(DriveBackBlueLeft.StartPose); 
            return DriveBackBlueLeft.buildAuto(driveSubsystem, trajectoryCommandFactory);
            case "DriveBackNegative": 
                driveSubsystem.setAutoStart(DriveBackNegative.StartPose); 
 return DriveBackNegative.buildAuto(driveSubsystem, trajectoryCommandFactory);
            case "Test": 
                driveSubsystem.setAutoStart(Test.StartPose); 
 return Test.buildAuto(driveSubsystem, trajectoryCommandFactory);

            case "PP Blue Right": return AutoBuilder.buildAuto("BlueRight");
            case "PP Blue Middle": return AutoBuilder.buildAuto("BlueMiddle");
            case "PP Blue Left": return AutoBuilder.buildAuto("BlueLeftAuto");
        }
        return null;
    }
}
