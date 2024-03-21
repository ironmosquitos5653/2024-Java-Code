        package frc.robot.commands.autos;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.TrajectoryCommandFactory;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;

public class AutoUtil {

    public static void addAutos(SendableChooser<String> chooser) {
        chooser.addOption("RedRightAuto", "RedRightAuto");
        chooser.addOption("BlueLeftAuto", "BlueLeftAuto");
        chooser.addOption("RedMiddle", "RedMiddle");
        chooser.addOption("BlueMiddle", "BlueMiddle");
        chooser.addOption("RedLeft", "RedLeft");
        chooser.addOption("BlueRight", "BlueRight");
        chooser.addOption("RedShoot", "RedShoot");
        chooser.addOption("BlueShoot", "BlueShoot");


        // PathPlanner autos
        // chooser.addOption("PP Blue Left", "PP Blue Left");
        // chooser.addOption("PP Blue Middle", "PP Blue Middle");
        // chooser.addOption("PP Blue Right", "PP Blue Right");
        // chooser.addOption("PP Red Left", "PP Blue Right");
        // chooser.addOption("PP Red Middle", "PP Blue Middle");
        // chooser.addOption("PP Red Right", "PP Blue Left");
    }

    public static Command getAuto(String name, DriveSubsystem driveSubsystem, TrajectoryCommandFactory trajectoryCommandFactory) {
        PoseEstimatorSubsystem.setVisionEnabled(false);
        switch(name) {
            case "RedShoot": 
                driveSubsystem.setAutoStart(RedShoot.StartPose); 
 return RedShoot.buildAuto(driveSubsystem, trajectoryCommandFactory);
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
            case "BlueShoot": 
                driveSubsystem.setAutoStart(BlueShoot.StartPose); 
 return BlueShoot.buildAuto(driveSubsystem, trajectoryCommandFactory);

            // case "PP Blue Right": return AutoBuilder.buildAuto("BlueRight");
            // case "PP Blue Middle": return AutoBuilder.buildAuto("BlueMiddle");
            // case "PP Blue Left": return AutoBuilder.buildAuto("BlueLeftAuto");
        }
        return null;
    }
}
