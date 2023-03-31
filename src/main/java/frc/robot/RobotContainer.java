// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OffsetFromTargetAprTag;
import frc.robot.commands.AprTagCommand;
import frc.robot.commands.ArmMoveCommands.MoveBucketCommand;
import frc.robot.commands.ArmMoveCommands.MoveGripperCommand;
import frc.robot.commands.ArmMoveCommands.MoveGripperCommand.GripperState;
import frc.robot.commands.ArmPositionCommands.*;
import frc.robot.commands.Auton.ThreeElement;
import frc.robot.commands.Auton.ThreeElementWMarkers;
import frc.robot.commands.Auton.line2meters;
import frc.robot.commands.Auton.line2metersCommand;
import frc.robot.commands.Auton.threeElement_Blue;
import frc.robot.commands.Auton.threeElement_Red;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.commands.GoToPickupTag;
import frc.robot.commands.GoToTagCommand;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.*;
import frc.robot.subsystems.ArmSubsystem.ArmPosition;
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
    public static final TimeOfFlightSubsystem timeOfFlightSubsystem = new TimeOfFlightSubsystem();

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

    // Vision
    // private final PhotonCamera photonCameraConeVision = new PhotonCamera(Constants.VisionConstants.coneCameraName);

    // private final ConeVisionSubsystem m_coneVisionSubsystem = new ConeVisionSubsystem(photonCameraConeVision);
    // private final PhotonCamera photonCamera = new PhotonCamera(Constants.VisionConstants.aprTagCameraName);
    public static final PhotonCamera photonCameraFront = new PhotonCamera(Constants.VisionConstants.aprTagCameraName);
    public static final PhotonCamera photonCameraBack =
            new PhotonCamera(Constants.VisionConstants.aprTagCameraBackName);

    // private final ConeVisionCommand  coneVisionCommand= new ConeVisionCommand(m_coneVisionSubsystem);

    // private final CubeVisionCommand  cubeVisionCommand= new CubeVisionCommand(m_coneVisionSubsystem);
    // private final PoseEstimatorSubSystemOld poseEstimator = new
    // PoseEstimatorSubSystemOld(photonCamera,swerveSubsystem);

    private final PoseEstimatorSubSystem poseEstimator =
            new PoseEstimatorSubSystem(swerveSubsystem::getRotation2d, swerveSubsystem::getModulePositions);

    // private final ChaseTagCommand chaseTagCommand = new
    // ChaseTagCommand(photonCamera,swerveSubsystem,poseEstimator::getCurrentPose, 6);
    private final AprTagCommand aprTagCommand =
            new AprTagCommand(photonCameraFront, m_exampleSubsystem, 8, poseEstimator::getCurrentPose);

    // Vision

    // Limelight

    // public static final LimeLightSubSystem limeLightSubSystem = new LimeLightSubSystem("limelight");
    //  private final PoseEstimatorSubSystem2 poseEstimator2 = new
    // PoseEstimatorSubSystem2(limeLightSubSystem,swerveSubsystem);

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
                () -> !driverJoystick.getRawButton(OIConstants.kDriverTopSpeed)));

        // armSubsystem.setDefaultCommand(new MicroAdjustCommand(
        //         armSubsystem,
        //         wristSubsystem,
        //         () -> -coDriverJoystick.getRawAxis(OIConstants.kDriverLeftYAxis),
        //         () -> -coDriverJoystick.getRawAxis(OIConstants.kDriverRightYAxis)));

        new MoveGripperCommand(gripperSubsystem, armSubsystem, MoveGripperCommand.GripperState.CLOSED).execute();
        new MoveBucketCommand(bucketSubsystem, MoveBucketCommand.BucketPosition.RETRACTED).execute();

        // Close the gripper the first time the item is detected
        new Trigger(() -> (armSubsystem.getArmPositionState() == ArmPosition.CUBE_PICKUP)
                        && (timeOfFlightSubsystem.getRange() < 140)
                        && (timeOfFlightSubsystem.getRange() > 30))
                .whileTrue(new MoveGripperCommand(gripperSubsystem, armSubsystem, GripperState.CLOSED));

        // Configure the button bindings
        configureButtonBindings();

        // chooser.addOption("Line 2 Meters", new line2meters(swerveSubsystem));
        // chooser.addOption("Line 2 Meters Turn", new line2metersTurn(swerveSubsystem));
        // chooser.addOption("Three Meters Vision Test", new threeMeterVisionTest(swerveSubsystem));
        // chooser.addOption("Three Meters Vision Test Command", new threeMeterVisionTestCommand(swerveSubsystem));
        // chooser.addOption("Three Element Command", new threeElementCommand(swerveSubsystem));

        // chooser.addOption("Five Meters Vision Test Command", new visionTest5M(swerveSubsystem, photonCamera,
        // m_exampleSubsystem, poseEstimator));
        // chooser.addOption("Five Meters Vision Test Command Marker", new visionTest5MMarker(swerveSubsystem,
        // photonCamera, m_exampleSubsystem, poseEstimator));

        // chooser.addOption("Two Cone Test With Vision", new twoConeWithVision(swerveSubsystem, photonCamera,
        // m_exampleSubsystem, poseEstimator));
        // // chooser.setDefaultOption("Five Meters With Vision", new  ParallelDeadlineGroup( new
        // AprTagCommand(photonCamera, m_exampleSubsystem, 8, poseEstimator::getCurrentPose),
        //     new visionTest5M(swerveSubsystem)).andThen(new GoToTagCommand(photonCamera, swerveSubsystem,
        // poseEstimator::getCurrentPose, 8)));

        // chooser.setDefaultOption("Line Three Meters With Vision Deadline", new  ParallelDeadlineGroup( new
        // AprTagCommand(photonCamera, m_exampleSubsystem, 8, poseEstimator::getCurrentPose),
        //                                           new threeMeterVisionTest(swerveSubsystem)).andThen(new
        // GoToTagCommand(photonCamera, swerveSubsystem, poseEstimator::getCurrentPose, 8)));

        chooser.addOption("Three Element Red", new threeElement_Red(swerveSubsystem));
        chooser.addOption("Three Element Blue", new threeElement_Blue(swerveSubsystem));
        chooser.addOption("Three Element with Markers", new ThreeElementWMarkers(swerveSubsystem));
        chooser.addOption("Marker Test", new Auton_2_Cone_Red(swerveSubsystem));


        chooser.addOption("Auto Three Element", new ThreeElement(swerveSubsystem));
        chooser.addOption("Line 2 Meters Command", new line2metersCommand(swerveSubsystem));
        chooser.addOption(
                "Line 2 Meters and Goto Tag",
                new SequentialCommandGroup(
                        new line2meters(swerveSubsystem),
                        new GoToTagCommand(photonCameraFront, swerveSubsystem, poseEstimator::getCurrentPose, 1),
                        new DriveToPoseCommand(
                                swerveSubsystem, poseEstimator::getCurrentPose, new Pose2d(0, 0, new Rotation2d()))));

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

        // Print April tag info to Smart dashboard - Button A
        new JoystickButton(driverJoystick, OIConstants.kDriverButtonA).onTrue(aprTagCommand);
        // Chase Aril Tag Continuous - Button B
        // new JoystickButton(driverJoystick, Constants.buttonB).onTrue(chaseTagCommand);
        // Go to Origin -  Button X
        // new JoystickButton(driverJoystick, Constants.buttonX).onTrue(
        //   new DriveToPoseCommand(swerveSubsystem,poseEstimator::getCurrentPose,new Pose2d(0, 0, new
        // Rotation2d().fromDegrees(90))));
        // Go to April Tag and Stop - Button Y

        new JoystickButton(driverJoystick, Constants.buttonY)
                .whileTrue(new GoToPickupTag(swerveSubsystem, swerveSubsystem::getPose));
        new JoystickButton(driverJoystick, Constants.buttonA)
                .whileTrue(new GoToTagCommand(photonCameraFront, swerveSubsystem,swerveSubsystem::getPose, 2, //Not used
                        OffsetFromTargetAprTag.CENTER));
        new JoystickButton(driverJoystick, Constants.buttonB)
                .whileTrue(new GoToTagCommand(
                        photonCameraFront, swerveSubsystem, swerveSubsystem::getPose, 2, OffsetFromTargetAprTag.LEFT));
        new JoystickButton(driverJoystick, Constants.buttonX)
                .whileTrue(new GoToTagCommand(
                        photonCameraFront, swerveSubsystem, swerveSubsystem::getPose, 2, OffsetFromTargetAprTag.RIGHT));

        // new JoystickButton(driverJoystick, Constants.buttonA).onTrue(new LimeLightTestCommand(limeLightSubSystem));

        // new JoystickButton(driverJoystick, Constants.buttonB).onTrue(new LimeLightAlignCommand(limeLightSubSystem,
        // swerveSubsystem));

        new Trigger(() -> driverJoystick.getRawAxis(OIConstants.kDriverRightTrigger) > 0.5
                        && armSubsystem.getArmPositionState() != ArmPosition.BUCKET_PICKUP)
                .onTrue(new MoveGripperCommand(gripperSubsystem, armSubsystem, GripperState.CLOSED));

        new Trigger(() -> driverJoystick.getRawAxis(OIConstants.kDriverLeftTrigger) > 0.5
                        && armSubsystem.getArmPositionState() != ArmPosition.BUCKET_PICKUP)
                .onTrue(new MoveGripperCommand(gripperSubsystem, armSubsystem, GripperState.OPENED));

        // home
        new Trigger(() -> coDriverJoystick.getPOV() > 180)
                .onTrue(new HomeCommand(armSubsystem, elevatorSubsystem, wristSubsystem, gripperSubsystem));

        // bucket pickup
        new JoystickButton(coDriverJoystick, OIConstants.kDriverButtonA)
                .onTrue(new BucketPickupCommand(
                        elevatorSubsystem, wristSubsystem, bucketSubsystem, armSubsystem, gripperSubsystem));

        // mid drop
        new JoystickButton(coDriverJoystick, OIConstants.kDriverButtonX)
                .onTrue(new MidDropCommand(armSubsystem, elevatorSubsystem, wristSubsystem));

        // high drop
        new JoystickButton(coDriverJoystick, OIConstants.kDriverButtonY)
                .onTrue(new HighDropCommand(armSubsystem, elevatorSubsystem, wristSubsystem));

        // ground drop
        new JoystickButton(coDriverJoystick, OIConstants.kDriverButtonB)
                .onTrue(new GroundDropCommand(armSubsystem, elevatorSubsystem, wristSubsystem));

        // ground pickup
        new JoystickButton(coDriverJoystick, OIConstants.kDriverLeftBumper)
                .onTrue(new GroundPickupCommand(elevatorSubsystem, wristSubsystem, armSubsystem, gripperSubsystem));

        // shelf pickup
        new JoystickButton(coDriverJoystick, OIConstants.kDriverRightBumper)
                .onTrue(new ShelfPickupCommand(armSubsystem, elevatorSubsystem, wristSubsystem));

        // cube pickup
        new Trigger(() -> coDriverJoystick.getRawAxis(OIConstants.kDriverRightTrigger) > 0.5)
                .onTrue(new CubePickupCommand(elevatorSubsystem, wristSubsystem, armSubsystem, gripperSubsystem));

        // Right trigger - close gripper when bucket pickup, and vice versa
        new Trigger(() -> (armSubsystem.getArmPositionState() == ArmPosition.BUCKET_PICKUP)
                        && (driverJoystick.getRawAxis(OIConstants.kDriverRightTrigger) > 0.5))
                .onTrue(new SequentialCommandGroup(
                        new MoveBucketCommand(bucketSubsystem, MoveBucketCommand.BucketPosition.RETRACTED),
                        new MoveGripperCommand(gripperSubsystem, armSubsystem, GripperState.CLOSED)));

        new Trigger(() -> (armSubsystem.getArmPositionState() == ArmPosition.BUCKET_PICKUP)
                        && (driverJoystick.getRawAxis(OIConstants.kDriverLeftTrigger) > 0.5))
                .onTrue(new SequentialCommandGroup(
                        new MoveBucketCommand(bucketSubsystem, MoveBucketCommand.BucketPosition.EXTENDED),
                        new MoveGripperCommand(gripperSubsystem, armSubsystem, GripperState.OPENED)));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return chooser.getSelected();
    }

    public static SwerveSubsystem getSwerveSubsystem() {
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

    public static BucketSubsystem getBucketSubsystem() {
        return bucketSubsystem;
    }

    public static TimeOfFlightSubsystem getTimeOfFlightSubsystem() {
        return timeOfFlightSubsystem;
    }
}
