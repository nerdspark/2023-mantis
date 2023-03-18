// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AprTagCommand;
import frc.robot.commands.ChaseTagCommand;
import frc.robot.commands.ConeVisionCommand;
import frc.robot.commands.CubeVisionCommand;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.commands.GoToTagCommand;
import frc.robot.commands.MicroAdjustCommand;
import frc.robot.commands.MoveArmCommand;
import frc.robot.commands.MoveBucketCommand;
import frc.robot.commands.MoveElevatorCommand;
import frc.robot.commands.MoveGripperCommand;
import frc.robot.commands.MoveWristCommand;
import frc.robot.commands.SwerveJoystickCmd;

import frc.robot.subsystems.ConeVisionSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.commands.Auton.ThreeElement;
import frc.robot.commands.Auton.line2meters;
import frc.robot.commands.Auton.line2metersCommand;
import frc.robot.commands.MoveGripperCommand.GripperState;
import frc.robot.subsystems.PoseEstimatorSubSystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.BucketSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmPosition;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

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
    public static final GripperSubsystem gripperSubsystem = new GripperSubsystem();
    public static final BucketSubsystem bucketSubsystem = new BucketSubsystem();
    public static final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    public static final WristSubsystem wristSubsystem = new WristSubsystem();

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
      () -> driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
      () -> -driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
      () -> driverJoystick.getRawAxis(OIConstants.kDriverRotXAxis),
      () -> driverJoystick.getRawAxis(OIConstants.kDriverRotYAxis),      
      () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx), 
      () -> driverJoystick.getPOV(), 
      () -> driverJoystick.getRawAxis(OIConstants.kDriverLeftTrigger), 
      () -> driverJoystick.getRawAxis(OIConstants.kDriverRightTrigger), 
      () -> driverJoystick.getRawButton(Constants.start), 
      () -> !driverJoystick.getRawButton(OIConstants.kDriverCancelTurn), 
      () -> driverJoystick.getRawButton(OIConstants.kDriverTopSpeed)));

    armSubsystem.setDefaultCommand(new MicroAdjustCommand(
      armSubsystem,
      wristSubsystem,
      () -> -coDriverJoystick.getRawAxis(OIConstants.kDriverLeftYAxis),   
      () -> -coDriverJoystick.getRawAxis(OIConstants.kDriverRightYAxis)
    ));

    // Configure the button bindings
    configureButtonBindings();

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
    new JoystickButton(driverJoystick, OIConstants.kDriverButtonA).onTrue(aprTagCommand);
    //Chase Aril Tag Continuous - Button B
    new JoystickButton(driverJoystick, OIConstants.kDriverButtonB).onTrue(chaseTagCommand);    
    //Go to Origin -  Button X
    new JoystickButton(driverJoystick, OIConstants.kDriverButtonX).onTrue(
      new DriveToPoseCommand(swerveSubsystem,poseEstimator::getCurrentPose,new Pose2d(0, 0, new Rotation2d())));
    //Go to April Tag and Stop - Button Y
    new JoystickButton(driverJoystick, OIConstants.kDriverButtonY).onTrue( 
      new  GoToTagCommand(photonCamera,swerveSubsystem,poseEstimator::getCurrentPose,1)); 
      
    // new JoystickButton(coDriverJoystick, OIConstants.kDriverButtonA).onTrue(homePositionCommand);
    // new JoystickButton(coDriverJoystick, OIConstants.kDriverButtonB).onTrue(scoreGroundCommand);

    new Trigger(() -> driverJoystick.getRawAxis(OIConstants.kDriverRightTrigger) > 0.5).onTrue(
        new MoveGripperCommand(gripperSubsystem, armSubsystem, GripperState.Closed));

    new Trigger(() -> driverJoystick.getRawAxis(OIConstants.kDriverLeftTrigger) > 0.5).onTrue(
      new MoveGripperCommand(gripperSubsystem, armSubsystem, GripperState.Open));

    // home
    new Trigger(() -> coDriverJoystick.getPOV() > 180).onTrue(new InstantCommand(() -> armSubsystem.setArmPositionState(ArmPosition.Home))).onTrue(
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                new MoveWristCommand(wristSubsystem, ArmConstants.homePosition.get("wristCmdPos")),
                new MoveElevatorCommand(elevatorSubsystem, ArmConstants.homePosition.get("inclinatorCmdPos"))),
            new MoveArmCommand(armSubsystem, ArmConstants.homePosition.get("armCmdPos"),
                ArmConstants.homePosition.get("smartMotionMaxVel"),
                ArmConstants.homePosition.get("smartMotionMaxAccel"))));

    // bucket pickup
    new JoystickButton(coDriverJoystick, OIConstants.kDriverButtonA).onTrue(new InstantCommand(() -> armSubsystem.setArmPositionState(ArmPosition.BucketPickup))).onTrue(
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                new MoveElevatorCommand(elevatorSubsystem, ArmConstants.intakeBucketPosition.get("inclinatorCmdPos")),
                new MoveWristCommand(wristSubsystem, ArmConstants.intakeBucketPosition.get("wristCmdPos")),
                new MoveBucketCommand(bucketSubsystem, MoveBucketCommand.BucketPosition.EXTENDED)),
            new MoveArmCommand(armSubsystem, ArmConstants.intakeBucketPosition.get("armCmdPos"),
                ArmConstants.intakeBucketPosition.get("smartMotionMaxVel"),
                ArmConstants.intakeBucketPosition.get("smartMotionMaxAccel"))));

    // score mid position
    new JoystickButton(coDriverJoystick, OIConstants.kDriverButtonX).onTrue(new InstantCommand(() -> armSubsystem.setArmPositionState(ArmPosition.MidDrop))).onTrue(
        new SequentialCommandGroup(
            new MoveArmCommand(armSubsystem, ArmConstants.scoreMidPosition.get("armCmdPos"),
                ArmConstants.scoreMidPosition.get("smartMotionMaxVel"),
                ArmConstants.scoreMidPosition.get("smartMotionMaxAccel")),
            new ParallelCommandGroup(
                new MoveElevatorCommand(elevatorSubsystem, ArmConstants.scoreMidPosition.get("inclinatorCmdPos")),
                new MoveWristCommand(wristSubsystem, ArmConstants.scoreMidPosition.get("wristCmdPos")))));
    
    // score high position
    new JoystickButton(coDriverJoystick, OIConstants.kDriverButtonY).onTrue(new InstantCommand(() -> armSubsystem.setArmPositionState(ArmPosition.HighDrop))).onTrue(
        new SequentialCommandGroup(
            new MoveArmCommand(armSubsystem, ArmConstants.scoreHighPosition.get("armCmdPos"),
                ArmConstants.scoreHighPosition.get("smartMotionMaxVel"),
                ArmConstants.scoreHighPosition.get("smartMotionMaxAccel")),
            new ParallelCommandGroup(
                new MoveElevatorCommand(elevatorSubsystem, ArmConstants.scoreHighPosition.get("inclinatorCmdPos")),
                new MoveWristCommand(wristSubsystem, ArmConstants.scoreHighPosition.get("wristCmdPos")))));

      // ground drop
      new JoystickButton(coDriverJoystick, OIConstants.kDriverButtonB).onTrue(new InstantCommand(() -> armSubsystem.setArmPositionState(ArmPosition.GroundDrop))).onTrue(
        new SequentialCommandGroup(
            new MoveArmCommand(armSubsystem, ArmConstants.scoreGroundPosition.get("armCmdPos"),
                ArmConstants.scoreGroundPosition.get("smartMotionMaxVel"),
                ArmConstants.scoreGroundPosition.get("smartMotionMaxAccel")),
            new ParallelCommandGroup(
                new MoveElevatorCommand(elevatorSubsystem, ArmConstants.scoreGroundPosition.get("inclinatorCmdPos")),
                new MoveWristCommand(wristSubsystem, ArmConstants.scoreGroundPosition.get("wristCmdPos")))));
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
