// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OffsetFromTargetAprTag;
import frc.robot.commands.AprTagCommand;
import frc.robot.commands.ChaseTagCommand;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.commands.GoToTagCommand;
import frc.robot.commands.ShelfPickupCommand;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.ArmMoveCommands.MoveBucketCommand;
import frc.robot.commands.ArmMoveCommands.MoveGripperCommand;
import frc.robot.commands.ArmMoveCommands.MoveGripperCommand.GripperState;
import frc.robot.commands.ArmPositionCommands.BucketPickupCommand;
import frc.robot.commands.ArmPositionCommands.GroundDropCommand;
import frc.robot.commands.ArmPositionCommands.GroundPickupCommand;
import frc.robot.commands.ArmPositionCommands.HomeCommand;
import frc.robot.commands.ArmPositionCommands.MicroAdjustCommand;
import frc.robot.commands.ArmPositionCommands.ScoreHighPositionCommand;
import frc.robot.commands.ArmPositionCommands.ScoreMidPositionCommand;
import frc.robot.subsystems.ConeVisionSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.commands.Auton.ThreeElement;
import frc.robot.commands.Auton.ThreeElementWMarkers;
import frc.robot.commands.Auton.line2meters;
import frc.robot.commands.Auton.line2metersCommand;
import frc.robot.commands.Auton.line2metersTurn;
import frc.robot.commands.Auton.threeElementCommand;
import frc.robot.commands.Auton.threeElement_Blue;
import frc.robot.commands.Auton.threeElement_Red;
import frc.robot.commands.Auton.threeMeterVisionTest;
import frc.robot.commands.Auton.threeMeterVisionTestCommand;
import frc.robot.commands.Auton.twoConeWithVision;
import frc.robot.commands.Auton.visionTest5M;
import frc.robot.commands.Auton.visionTest5MMarker;
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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.function.BooleanSupplier;

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

    BooleanSupplier hasCone;
    BooleanSupplier hasCube;

     
    Command autonomousCommand;
    SendableChooser<Command> chooser = new SendableChooser<>();

  // //Arm, Gripper, Bucket
  //   private static final ArmSubsystem armSubsystem = new ArmSubsystem();
    // private static final GripperSubsystem gripperSubsystem = new GripperSubsystem(0, 0, false, false);
  //   private static final BucketSubsystem bucketSubsystem = new BucketSubsystem();

  //Vision
  // private final PhotonCamera photonCameraConeVision = new PhotonCamera(Constants.VisionConstants.coneCameraName);

  // private final ConeVisionSubsystem m_coneVisionSubsystem = new ConeVisionSubsystem(photonCameraConeVision);
  private final PhotonCamera photonCamera = new PhotonCamera(Constants.VisionConstants.aprTagCameraName);

  // private final ConeVisionCommand  coneVisionCommand= new ConeVisionCommand(m_coneVisionSubsystem);

  // private final CubeVisionCommand  cubeVisionCommand= new CubeVisionCommand(m_coneVisionSubsystem);
  private final PoseEstimatorSubSystem poseEstimator = new PoseEstimatorSubSystem(photonCamera,swerveSubsystem);
  // private final ChaseTagCommand chaseTagCommand = new ChaseTagCommand(photonCamera,swerveSubsystem,poseEstimator::getCurrentPose, 6);
  private final AprTagCommand aprTagCommand = new AprTagCommand(photonCamera,m_exampleSubsystem,8,poseEstimator::getCurrentPose);

  
  //Vision

  //Limelight

  // public static final LimeLightSubSystem limeLightSubSystem = new LimeLightSubSystem("limelight");
  //  private final PoseEstimatorSubSystem2 poseEstimator2 = new PoseEstimatorSubSystem2(limeLightSubSystem,swerveSubsystem);



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
      swerveSubsystem,
      () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
      () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
      () -> -driverJoystick.getRawAxis(OIConstants.kDriverRotXAxis),
      () -> -driverJoystick.getRawAxis(OIConstants.kDriverRotYAxis),      
      () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx), 
      () -> driverJoystick.getPOV(), 
      () -> driverJoystick.getRawAxis(OIConstants.kDriverLeftTrigger), 
      () -> driverJoystick.getRawAxis(OIConstants.kDriverRightTrigger), 
      () -> driverJoystick.getRawButton(Constants.start), 
      () -> driverJoystick.getRawButton(OIConstants.kDriverCancelTurn), 
      () -> driverJoystick.getRawButton(OIConstants.kDriverTopSpeed)));

    armSubsystem.setDefaultCommand(new MicroAdjustCommand(
      armSubsystem,
      wristSubsystem,
      () -> -coDriverJoystick.getRawAxis(OIConstants.kDriverLeftYAxis),   
      () -> -coDriverJoystick.getRawAxis(OIConstants.kDriverRightYAxis)
    ));

    new MoveGripperCommand(gripperSubsystem, armSubsystem, MoveGripperCommand.GripperState.Closed).execute();

    // Configure the button bindings
    configureButtonBindings();

    // chooser.addOption("Line 2 Meters", new line2meters(swerveSubsystem));
    // chooser.addOption("Line 2 Meters Turn", new line2metersTurn(swerveSubsystem));
    // chooser.addOption("Three Meters Vision Test", new threeMeterVisionTest(swerveSubsystem));
    // chooser.addOption("Three Meters Vision Test Command", new threeMeterVisionTestCommand(swerveSubsystem));
    // chooser.addOption("Three Element Command", new threeElementCommand(swerveSubsystem));

    // chooser.addOption("Five Meters Vision Test Command", new visionTest5M(swerveSubsystem, photonCamera, m_exampleSubsystem, poseEstimator));
    // chooser.addOption("Five Meters Vision Test Command Marker", new visionTest5MMarker(swerveSubsystem, photonCamera, m_exampleSubsystem, poseEstimator));

    // chooser.addOption("Two Cone Test With Vision", new twoConeWithVision(swerveSubsystem, photonCamera, m_exampleSubsystem, poseEstimator));
    // // chooser.setDefaultOption("Five Meters With Vision", new  ParallelDeadlineGroup( new AprTagCommand(photonCamera, m_exampleSubsystem, 8, poseEstimator::getCurrentPose),
    //     new visionTest5M(swerveSubsystem)).andThen(new GoToTagCommand(photonCamera, swerveSubsystem, poseEstimator::getCurrentPose, 8)));

    // chooser.setDefaultOption("Line Three Meters With Vision Deadline", new  ParallelDeadlineGroup( new AprTagCommand(photonCamera, m_exampleSubsystem, 8, poseEstimator::getCurrentPose),
    //                                           new threeMeterVisionTest(swerveSubsystem)).andThen(new GoToTagCommand(photonCamera, swerveSubsystem, poseEstimator::getCurrentPose, 8)));

    chooser.addOption("Three Element Red", new threeElement_Red(swerveSubsystem));
    chooser.addOption("Three Element Blue", new threeElement_Blue(swerveSubsystem));
    chooser.addOption("Three Element with Markers", new ThreeElementWMarkers(swerveSubsystem));

    chooser.addOption("Auto Three Element", new ThreeElement(swerveSubsystem));
    chooser.addOption("Line 2 Meters Command", new line2metersCommand(swerveSubsystem));
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
    // new JoystickButton(driverJoystick, Constants.buttonB).onTrue(chaseTagCommand);    
    //Go to Origin -  Button X
    // new JoystickButton(driverJoystick, Constants.buttonX).onTrue(
    //   new DriveToPoseCommand(swerveSubsystem,poseEstimator::getCurrentPose,new Pose2d(0, 0, new Rotation2d().fromDegrees(90))));
    //Go to April Tag and Stop - Button Y
      new JoystickButton(driverJoystick, Constants.buttonY).onTrue(new GoToTagCommand(photonCamera, swerveSubsystem,poseEstimator::getCurrentPose , 4, OffsetFromTargetAprTag.CENTER));
      new JoystickButton(driverJoystick, Constants.buttonA).onTrue(new GoToTagCommand(photonCamera, swerveSubsystem,poseEstimator::getCurrentPose , 2, OffsetFromTargetAprTag.CENTER));
      new JoystickButton(driverJoystick, Constants.buttonB).onTrue(new GoToTagCommand(photonCamera, swerveSubsystem,poseEstimator::getCurrentPose , 2, OffsetFromTargetAprTag.LEFT));
      new JoystickButton(driverJoystick, Constants.buttonX).onTrue(new GoToTagCommand(photonCamera, swerveSubsystem,poseEstimator::getCurrentPose , 2, OffsetFromTargetAprTag.RIGHT));

    // new JoystickButton(driverJoystick, Constants.buttonA).onTrue(new LimeLightTestCommand(limeLightSubSystem));

    // new JoystickButton(driverJoystick, Constants.buttonB).onTrue(new LimeLightAlignCommand(limeLightSubSystem, swerveSubsystem));

    new Trigger(() -> driverJoystick.getRawAxis(OIConstants.kDriverRightTrigger) > 0.5 && armSubsystem.getArmPositionState() != ArmPosition.BucketPickup).onTrue(
        new MoveGripperCommand(gripperSubsystem, armSubsystem, GripperState.Closed));

    new Trigger(() -> driverJoystick.getRawAxis(OIConstants.kDriverLeftTrigger) > 0.5 && armSubsystem.getArmPositionState() != ArmPosition.BucketPickup).onTrue(
      new MoveGripperCommand(gripperSubsystem, armSubsystem, GripperState.Open));

    // home
    new Trigger(() -> coDriverJoystick.getPOV() > 180)
        .onTrue(new InstantCommand(() -> armSubsystem.setArmPositionState(ArmPosition.Home)))
        .onTrue(new HomeCommand(armSubsystem, elevatorSubsystem, wristSubsystem, gripperSubsystem));

    // bucket pickup
    new JoystickButton(coDriverJoystick, OIConstants.kDriverButtonA)
        .onTrue(new InstantCommand(() -> armSubsystem.setArmPositionState(ArmPosition.BucketPickup)))
        .onTrue(new BucketPickupCommand(elevatorSubsystem, wristSubsystem, bucketSubsystem, armSubsystem, gripperSubsystem));

    // score mid position
    new JoystickButton(coDriverJoystick, OIConstants.kDriverButtonX)
        .onTrue(new InstantCommand(() -> armSubsystem.setArmPositionState(ArmPosition.MidDrop)))
        .onTrue(new ScoreMidPositionCommand(armSubsystem, elevatorSubsystem, wristSubsystem));

    // score high position
    new JoystickButton(coDriverJoystick, OIConstants.kDriverButtonY)
        .onTrue(new InstantCommand(() -> armSubsystem.setArmPositionState(ArmPosition.HighDrop)))
        .onTrue(new ScoreHighPositionCommand(armSubsystem, elevatorSubsystem, wristSubsystem));

    // ground drop
    new JoystickButton(coDriverJoystick, OIConstants.kDriverButtonB)
        .onTrue(new InstantCommand(() -> armSubsystem.setArmPositionState(ArmPosition.GroundDrop)))
        .onTrue(new GroundDropCommand(armSubsystem, elevatorSubsystem, wristSubsystem));

    // ground pickup
    new JoystickButton(coDriverJoystick, OIConstants.kDriverLeftBumper)
        .onTrue(new InstantCommand(() -> armSubsystem.setArmPositionState(ArmPosition.GroundPickup)))
        .onTrue(new GroundPickupCommand(elevatorSubsystem, wristSubsystem, armSubsystem, gripperSubsystem));

    // shelf pickup
    new JoystickButton(coDriverJoystick, OIConstants.kDriverRightBumper)
        .onTrue(new InstantCommand(() -> armSubsystem.setArmPositionState(ArmPosition.ShelfPickup)))
        .onTrue(new ShelfPickupCommand(armSubsystem, elevatorSubsystem, wristSubsystem));

    // Right trigger - close gripper when bucket pickup, and vice versa
    new Trigger(() -> (armSubsystem.getArmPositionState() == ArmPosition.BucketPickup)
        && (driverJoystick.getRawAxis(OIConstants.kDriverRightTrigger) > 0.5))
        .onTrue(new SequentialCommandGroup(
          new MoveBucketCommand(bucketSubsystem, MoveBucketCommand.BucketPosition.RETRACTED),
          new MoveGripperCommand(gripperSubsystem, armSubsystem, GripperState.Closed)
        ));

    new Trigger(() -> (armSubsystem.getArmPositionState() == ArmPosition.BucketPickup)
        && (driverJoystick.getRawAxis(OIConstants.kDriverLeftTrigger) > 0.5))
        .onTrue(new SequentialCommandGroup(
          new MoveBucketCommand(bucketSubsystem, MoveBucketCommand.BucketPosition.EXTENDED),
          new MoveGripperCommand(gripperSubsystem, armSubsystem, GripperState.Open)
        ));
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

  public static ArmSubsystem getArmSubsystem() {
    return armSubsystem;
  }

  public static WristSubsystem getWristSubsystem() {
    return wristSubsystem;
  }

  public static ElevatorSubsystem getElevatorSubsystem() {
    return elevatorSubsystem;
  }

  public static GripperSubsystem getGripperSubsystem() {
    return gripperSubsystem;
  }
}
