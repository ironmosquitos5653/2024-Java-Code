// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AmpShootCommand;
import frc.robot.commands.AmpUpCommand;
import frc.robot.commands.ClimbDownCommand;
import frc.robot.commands.ClimbUpCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeNoteCommand;
import frc.robot.commands.IntakeSpitCommand;
import frc.robot.commands.ShootAutoCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.autos.AutoUtil;
import frc.robot.commands.autos.BlueLeftAuto;
import frc.robot.commands.autos.BlueMiddle;
import frc.robot.commands.autos.BlueRight;
import frc.robot.commands.autos.DriveBackBlueLeft;
import frc.robot.commands.autos.DriveBackRedRight;
import frc.robot.commands.autos.RedLeft;
import frc.robot.commands.autos.RedMiddle;
import frc.robot.commands.autos.RedRightAuto;
import frc.robot.commands.autos.Test1Blue;
import frc.robot.commands.autos.Test1Red;
import frc.robot.subsystems.AmpSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LifterSubsystem;
import frc.robot.subsystems.MoveAmpSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final PoseEstimatorSubsystem m_PoseEstimatorSubsystem = new PoseEstimatorSubsystem(m_robotDrive);
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  public final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  private final LifterSubsystem m_LifterSubsystem = new LifterSubsystem(m_PoseEstimatorSubsystem);
  private final AmpSubsystem m_AmpSubsystem = new AmpSubsystem();
  private final MoveAmpSubsystem m_MoveAmpSubsystem = new MoveAmpSubsystem();
  private final ClimbSubsystem m_ClimbSubsystem = new ClimbSubsystem();
  private final AutonomousManager m_AutonomousManager = new AutonomousManager(m_IntakeSubsystem, m_LifterSubsystem, m_ShooterSubsystem);
  private final TrajectoryCommandFactory m_TrajectoryCommandFactory = new TrajectoryCommandFactory(m_robotDrive);



  // The driver's controller
 // XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_driverController = new CommandXboxController(0);

  private SendableChooser<String> sendableChooser;
  private Map<String, Command> autoCommands = new HashMap<String, Command>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_robotDrive.setPoseEstimator(m_PoseEstimatorSubsystem);
    // Configure the button bindings
    configureButtonBindings();

    m_PoseEstimatorSubsystem.setDefaultCommand(new RunCommand( () -> {}, m_PoseEstimatorSubsystem));

    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));

    m_AutonomousManager.initialize();

    SmartDashboard.putData(new ShootAutoCommand(m_ShooterSubsystem, m_IntakeSubsystem, .5));
    
    buildAutos();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // new JoystickButton(m_driverController, Button.kR1.value)
    //     .whileTrue(new RunCommand(
    //         () -> m_robotDrive.setX(),
    //         m_robotDrive));
    m_driverController.a().onTrue(new IntakeCommand(m_IntakeSubsystem));
    m_driverController.b().whileTrue(new IntakeSpitCommand(m_IntakeSubsystem));

    /* from speaker */
    m_driverController.rightBumper().onTrue(Commands.runOnce(() -> m_LifterSubsystem.setAngle(m_PoseEstimatorSubsystem.getVerticalShootAngle())) //53
      .andThen(new ShootCommand(m_ShooterSubsystem, m_IntakeSubsystem, m_LifterSubsystem, m_robotDrive, .5)));

    /* from stage */
    m_driverController.leftBumper().onTrue(Commands.runOnce(() -> m_LifterSubsystem.setAngle(40))
    .andThen(new ShootCommand(m_ShooterSubsystem, m_IntakeSubsystem, m_LifterSubsystem, m_robotDrive,.5)));

    m_driverController.povUp().onTrue(new IntakeNoteCommand(m_AmpSubsystem, m_IntakeSubsystem));
    m_driverController.povRight().onTrue(new AmpUpCommand(m_IntakeSubsystem, m_MoveAmpSubsystem));
    m_driverController.povDown().onTrue(new AmpShootCommand(m_AmpSubsystem, m_MoveAmpSubsystem));
    m_driverController.povLeft().onTrue(Commands.runOnce(() -> m_robotDrive.zeroHeading()));

    m_driverController.y().whileTrue(new ClimbUpCommand(m_ClimbSubsystem, 1));
    m_driverController.x().whileTrue(new ClimbDownCommand(m_ClimbSubsystem, 1));

    SmartDashboard.putData("xxx", Commands.runOnce(() -> m_robotDrive.setAutoAim(true), m_robotDrive));
  }

    
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommandX() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
  }

  public Command getAutonomousCommand() {
    Command auto = AutoUtil.getAuto(sendableChooser.getSelected(), m_robotDrive, m_TrajectoryCommandFactory);

    return auto;
  }

  public void buildAutos() {
    sendableChooser = new SendableChooser<String>();
    AutoUtil.addAutos(sendableChooser);

    // addAuto("DriveBack", AutoBuilder.buildAuto("DriveBack"));
    // addAuto("BankAndForth", AutoBuilder.buildAuto("Back and forth"));
    // addAuto("test", AutoBuilder.buildAuto("test"));
    // addAuto("TwoNote2", AutoBuilder.buildAuto("TwoNote2"));
    // addAuto("TwoNote3", AutoBuilder.buildAuto("TwoNote3"));
     //addAuto("BlueRight", AutoBuilder.buildAuto("BlueRight"));
    // addAuto("ThreeNote2", AutoBuilder.buildAuto("ThreeNote2"));
    // addAuto("BlueLeftAuto", AutoBuilder.buildAuto("BlueLeftAuto"));
    // addAuto("RedLeftAuto", AutoBuilder.buildAuto("RedLeftAuto"));
    // addAuto("BlueMiddle", AutoBuilder.buildAuto("BlueMiddle"));
    // addAuto("DriveBackNegative", AutoBuilder.buildAuto("DriveBackNegative"));

    //  sendableChooser.addOption("RedMiddle", "RedMiddle");
    //  sendableChooser.addOption("RedLeft", "RedLeft");

    SmartDashboard.putData("Autonomous Chooser", sendableChooser);
  }

  public void addAuto(String name, Command command) {
    autoCommands.put(name, command);
    sendableChooser.addOption(name, name);

  }
}
