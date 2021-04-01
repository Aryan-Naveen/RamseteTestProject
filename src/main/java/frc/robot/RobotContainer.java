// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.preferences.AutoOption;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  private Trajectory trajectory = new Trajectory();

  private SendableChooser<AutoOption> chooser = new SendableChooser<>();

  private double kS = 0.0843;
  private double kV = 0.239;
  private double kA = 0.0175;

  private Path slalomPath;
  private Path pathRedA;
  private Path pathRedB;
  private Path pathBlueA;
  private Path pathBlueB;
  private Path trajectoryPath;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    chooser.setDefaultOption("Slalom", AutoOption.Slalom);
    chooser.addOption("Red A", AutoOption.RedA);
    chooser.addOption("Red B", AutoOption.RedB);
    chooser.addOption("Blue A", AutoOption.BlueA);
    chooser.addOption("Blue B", AutoOption.BlueB);
    SmartDashboard.putData("auto paths", chooser);

    // Configure the button bindings
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(kS,
                                       Constants.kvVoltSecondsPerMeter,
                                       Constants.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveKinematics,
            12);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
                          Constants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);    
    configureButtonBindings();

    String slalomJson = "paths/circle.wpilib.json";
    slalomPath = Filesystem.getDeployDirectory().toPath().resolve(slalomJson);



    String pathRedAJson = "paths/PathRedA.wpilib.json";
    pathRedA = Filesystem.getDeployDirectory().toPath().resolve(pathRedAJson);

    String pathRedBJson = "paths/PathRedB.wpilib.json";
    pathRedB = Filesystem.getDeployDirectory().toPath().resolve(pathRedBJson);

    String pathBlueAJson = "paths/PathBlueA.wpilib.json";
    pathBlueA = Filesystem.getDeployDirectory().toPath().resolve(pathBlueAJson);

    String pathBlueBJson = "paths/PathBlueB.wpilib.json";
    pathBlueB = Filesystem.getDeployDirectory().toPath().resolve(pathBlueBJson);
}


  private Translation2d convertToMeters(Translation2d translation){
    return translation.times(0.0254);
  }

  public void arcadedrive(Joystick joystick){
    m_robotDrive.arcadeDrive(-joystick.getY(), joystick.getX());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // Create a voltage constraint to ensure we don't accelerate too fast

    AutoOption choice = chooser.getSelected();
    if(choice == AutoOption.Slalom){
      SmartDashboard.putNumber("Choice", 0);
    }


    switch(choice){
      case Slalom:
          trajectoryPath = slalomPath;
          break;
      case RedA:
          trajectoryPath = pathRedA;
          break;
      case RedB:
          trajectoryPath = pathRedB;
          break;
      case BlueA:
          trajectoryPath = pathBlueA;
          break;
      case BlueB:
          trajectoryPath = pathBlueB;
          break;
    }
    try {
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("ERROR", ex.getStackTrace());
    }

 
    RamseteCommand ramseteCommand = new RamseteCommand(
      trajectory,
        m_robotDrive::getPose,
        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
        new SimpleMotorFeedforward(Constants.ksVolts,
                                    Constants.kvVoltSecondsPerMeter,
                                    Constants.kaVoltSecondsSquaredPerMeter),
        Constants.kDriveKinematics,
        m_robotDrive::getWheelSpeeds,
        new PIDController(Constants.kPDriveVel, 0, 0),
        new PIDController(Constants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_robotDrive::tankDriveVolts,
        m_robotDrive
    );

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(trajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
  }

  public void initializeDrive(){
    m_robotDrive.init();
  }
}
