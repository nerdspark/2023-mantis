// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.Constants.OIConstants;
import frc.robot.commands.AprTagCommand;
import frc.robot.commands.ChaseTagCommand;
import frc.robot.commands.ConeVisionCommand;
import frc.robot.commands.CubeVisionCommand;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.commands.GoToTagCommand;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.ArmJoystickCmd;
import frc.robot.subsystems.ConeVisionSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.commands.Auton.ThreeElement;
import frc.robot.commands.Auton.line2meters;
import frc.robot.commands.Auton.line2metersCommand;
import frc.robot.subsystems.PoseEstimatorSubSystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.photonvision.PhotonCamera;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  // private final CommandXboxController m_driverController =
  //     new CommandXboxController(OperatorConstants.kDriverControllerPort);

    private static final XboxController cont = new XboxController(Constants.controllerPort);

    private static final XboxController cont2 = new XboxController(Constants.controllerPort2);

    public static final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    public static final ArmSubsystem armSubsystem = new ArmSubsystem();

    private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);

    private final Joystick coDriverJoystick = new Joystick(OIConstants.kCoDriverControllerPort);

    public static final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

      
    Command autonomousCommand;
    SendableChooser<Command> chooser = new SendableChooser<>();

  //Vision
  private final PhotonCamera photonCameraConeVision = new PhotonCamera(Constants.VisionConstants.coneCameraName);

  private final ConeVisionSubsystem m_coneVisionSubsystem = new ConeVisionSubsystem(photonCameraConeVision);
  private final PhotonCamera photonCamera = new PhotonCamera(Constants.VisionConstants.aprTagCameraName);

  private final ConeVisionCommand  coneVisionCommand= new ConeVisionCommand(m_coneVisionSubsystem);

  private final CubeVisionCommand  cubeVisionCommand= new CubeVisionCommand(m_coneVisionSubsystem);
  private final PoseEstimatorSubSystem poseEstimator = new PoseEstimatorSubSystem( photonCamera,swerveSubsystem);
  private final ChaseTagCommand chaseTagCommand = new ChaseTagCommand(photonCamera,swerveSubsystem,poseEstimator::getCurrentPose, 8);
  private final AprTagCommand aprTagCommand = new AprTagCommand(photonCamera,m_exampleSubsystem,1,poseEstimator::getCurrentPose);

  // private final DriveToPoseCommand estimatePoseCommand = new DriveToPoseCommand(swerveSubsystem,poseEstimator::getCurrentPose);


  //Vision

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
      swerveSubsystem,
      () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
      () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
      () -> driverJoystick.getRawAxis(OIConstants.kDriverRotXAxis),
      () -> driverJoystick.getRawAxis(OIConstants.kDriverRotYAxis),      
      () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx), 
      () -> driverJoystick.getPOV(), 
      () -> driverJoystick.getRawAxis(OIConstants.kDriverLeftTrigger), 
      () -> driverJoystick.getRawAxis(OIConstants.kDriverRightTrigger), 
      () -> driverJoystick.getRawButton(Constants.buttonY), 
      () -> driverJoystick.getRawButton(OIConstants.kDriverCancelTurn), 
      () -> driverJoystick.getRawButton(OIConstants.kDriverTopSpeed)));

    // Configure the button bindings
    configureButtonBindings();

    armSubsystem.setDefaultCommand(new ArmJoystickCmd(
      armSubsystem,
      () -> -coDriverJoystick.getRawAxis(OIConstants.kDriverYAxis),
      () -> coDriverJoystick.getRawAxis(OIConstants.kDriverXAxis),
      () -> coDriverJoystick.getRawAxis(OIConstants.kDriverRotXAxis),
      () -> coDriverJoystick.getRawAxis(OIConstants.kDriverRotYAxis),      
      () -> !coDriverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx), 
      () -> coDriverJoystick.getPOV(), 
      () -> coDriverJoystick.getRawAxis(OIConstants.kDriverLeftTrigger), 
      () -> coDriverJoystick.getRawAxis(OIConstants.kDriverRightTrigger), 
      () -> coDriverJoystick.getRawButton(Constants.buttonY), 
      () -> coDriverJoystick.getRawButton(OIConstants.kDriverCancelTurn), 
      () -> coDriverJoystick.getRawButton(OIConstants.kDriverTopSpeed)));

    chooser.setDefaultOption("Line 2 Meters", new line2meters(swerveSubsystem));
    chooser.addOption("Auto Three Element", new ThreeElement(swerveSubsystem));
    chooser.addOption("Line 2 Meters Command", new line2metersCommand());
    chooser.addOption("Line 2 Meters and Goto Tag", new  SequentialCommandGroup( new line2meters(swerveSubsystem), 
                                              new GoToTagCommand(photonCamera, swerveSubsystem, poseEstimator::getCurrentPose, 1),
                                              new DriveToPoseCommand(swerveSubsystem, poseEstimator::getCurrentPose, new Pose2d(0, 0, new Rotation2d()))));

    Shuffleboard.getTab("Autonomous").add(chooser);
  }

    /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureButtonBindings() {

    SmartDashboard.putString("Config", "Button Bindings");

    //Print April tag info to Smart dashboard - Button A
    new JoystickButton(driverJoystick, Constants.buttonA).onTrue(aprTagCommand);
    //Chase Aril Tag Continuous - Button B
    new JoystickButton(driverJoystick, Constants.buttonB).onTrue(chaseTagCommand);    
    //Go to Origin -  Button X
    new JoystickButton(driverJoystick, Constants.buttonX).onTrue(
      new DriveToPoseCommand(swerveSubsystem,poseEstimator::getCurrentPose,new Pose2d(0, 0, new Rotation2d())));
    //Go to April Tag and Stop - Button Y
      new JoystickButton(driverJoystick, Constants.buttonY).onTrue( 
        new  GoToTagCommand(photonCamera,swerveSubsystem,poseEstimator::getCurrentPose,1));    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }

  public static SwerveSubsystem getSwerveSubsystem(){
    return swerveSubsystem;
  }

  public static ArmSubsystem getArmSubsystem(){
    return armSubsystem;
  }
}
