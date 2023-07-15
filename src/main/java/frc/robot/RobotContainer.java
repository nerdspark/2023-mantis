// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ArmMoveCommands.MoveBucketCommand;
import frc.robot.commands.ArmMoveCommands.MoveGripperCommand;
import frc.robot.commands.ArmMoveCommands.MoveGripperCommand.GripperState;
import frc.robot.commands.ArmMoveCommands.MoveWristCommand;
import frc.robot.commands.ArmPositionCommands.*;
import frc.robot.commands.Auton.*;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.*;
import frc.robot.subsystems.ArmSubsystem.ArmPosition;
import frc.robot.subsystems.BucketSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TimeOfFlightSubsystem;
import frc.robot.subsystems.WristSubsystem;
// import org.photonvision.PhotonCamera;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

    public static final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    public static final LimelightSubsystem limeLightSubsystem = new LimelightSubsystem();

    public static final ArmSubsystem armSubsystem = new ArmSubsystem();
    public static final GripperSubsystem gripperSubsystem = new GripperSubsystem();
    public static final BucketSubsystem bucketSubsystem = new BucketSubsystem();
    public static final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    public static final WristSubsystem wristSubsystem = new WristSubsystem();
    public static final TimeOfFlightSubsystem timeOfFlightSubsystem = new TimeOfFlightSubsystem();

    private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);

    private final Joystick coDriverJoystick = new Joystick(OIConstants.kCoDriverControllerPort);
    SendableChooser<Command> chooser = new SendableChooser<>();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Configure the trigger bindings
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                limeLightSubsystem,
                () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
                () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
                () -> -driverJoystick.getRawAxis(OIConstants.kDriverRotXAxis),
                () -> -driverJoystick.getRawAxis(OIConstants.kDriverRotYAxis),
                () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx),
                () -> driverJoystick.getPOV(),
                () -> driverJoystick.getRawAxis(OIConstants.kDriverLeftTrigger),
                () -> driverJoystick.getRawAxis(OIConstants.kDriverRightTrigger),
                () -> driverJoystick.getRawButton(OIConstants.kDriverStart),
                () -> driverJoystick.getRawButton(OIConstants.kDriverCancelTurn),
                () -> !driverJoystick.getRawButton(OIConstants.kDriverTopSpeed),
                () -> driverJoystick.getRawButton(OIConstants.kDriverRightBumper)));

        armSubsystem.setDefaultCommand(new MicroAdjustCommand(
                armSubsystem,
                wristSubsystem,
                () -> -coDriverJoystick.getRawAxis(Math.abs(OIConstants.kDriverLeftYAxis) > 0.1? OIConstants.kDriverLeftYAxis : 0),
                () -> -coDriverJoystick.getRawAxis(Math.abs(OIConstants.kDriverRightYAxis) > 0.1? OIConstants.kDriverRightYAxis : 0)));

        new MoveGripperCommand(gripperSubsystem, armSubsystem, MoveGripperCommand.GripperState.CLOSED).execute();
        new MoveBucketCommand(bucketSubsystem, MoveBucketCommand.BucketPosition.RETRACTED).execute();
        new MoveWristCommand(wristSubsystem, Constants.ArmConstants.homePosition.wristCmdPos()).execute();

        // Close the gripper the first time the item is detected
        new Trigger(() -> (armSubsystem.getArmPositionState() == ArmPosition.CUBE_PICKUP)
                        && (timeOfFlightSubsystem.lastValuesWithinBounds(10, 400))
                        && timeOfFlightSubsystem.getRange() < 180)
                .whileTrue(new MoveGripperCommand(gripperSubsystem, armSubsystem, GripperState.CLOSED));

        // Configure the button bindings
        configureButtonBindings();

        chooser.addOption("2 Cone Red", new threeElement_Red(swerveSubsystem, limeLightSubsystem));
        chooser.addOption("2 Cone Blue", new threeElement_Blue(swerveSubsystem, limeLightSubsystem));
        chooser.addOption("3 Cone Red", new RedBobFourCone());
        chooser.addOption("3 Cone Blue", new BlueBobFourCone());
        //        chooser.addOption("Charge Station Balance", new RedSideCharge());
        chooser.addOption(
                "Charge Station Balance", new StopRobotWait(swerveSubsystem).deadlineWith(new RedSideCharge2()));
        chooser.addOption("Red Side Bump", new RedSidePath());
        chooser.addOption("Blue Side Bump", new BlueSidePath());
        //        chooser.addOption("Three Element with Markers", new ThreeElementWMarkers(swerveSubsystem));
        //
        //        chooser.addOption("Auto Three Element", new ThreeElement(swerveSubsystem));
        //        chooser.addOption("Line 2 Meters Command", new line2metersCommand(swerveSubsystem));
        //        chooser.addOption(
        //                "Line 2 Meters and Goto Tag",
        //                new SequentialCommandGroup(
        //                        new line2meters(swerveSubsystem),
        //                        new GoToTagCommand(photonCamera, swerveSubsystem, poseEstimator::getCurrentPose, 1),
        //                        new DriveToPoseCommand(
        //                                swerveSubsystem, poseEstimator::getCurrentPose, new Pose2d(0, 0, new
        // Rotation2d()))));

        Shuffleboard.getTab("Autonomous").add(chooser);

        //        configureDashboard();
    }

    //    private void configureDashboard() {
    //        /**** Driver tab ****/
    //        var driverTab = Shuffleboard.getTab("Driver");
    //
    //        driverTab
    //                .add(new HttpCamera(VisionConstants.aprTagCameraName, "https://Photonvision.local:1181"))
    //                .withWidget(BuiltInWidgets.kCameraStream)
    //                .withProperties(Map.of("showCrosshair", true, "showControls", false, "rotation", "QUARTER_CCW"))
    //                .withSize(4, 6)
    //                .withPosition(0, 0);
    //
    //        /**** Vision tab ****/
    //        final var visionTab = Shuffleboard.getTab("Vision");
    //
    //        // Pose estimation
    //        poseEstimator.addDashboardWidgets(visionTab);
    //    }

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
        // new JoystickButton(driverJoystick, OIConstants.kDriverButtonA).onTrue(aprTagCommand);
        // Chase Aril Tag Continuous - Button B
        // new JoystickButton(driverJoystick, Constants.buttonB).onTrue(chaseTagCommand);
        // Go to Origin -  Button X
        // new JoystickButton(driverJoystick, Constants.buttonX).onTrue(
        //   new DriveToPoseCommand(swerveSubsystem,poseEstimator::getCurrentPose,new Pose2d(0, 0, new
        // Rotation2d().fromDegrees(90))));
        // Go to April Tag and Stop - Button Y
        //        new JoystickButton(driverJoystick, Constants.buttonY)
        //                .whileTrue(new GoToTagCommand(
        //                        photonCamera, swerveSubsystem, swerveSubsystem::getPose, 4,
        // OffsetFromTargetAprTag.CENTER));
        //        new JoystickButton(driverJoystick, Constants.buttonA)
        //                .whileTrue(new GoToTagCommand(
        //                        photonCamera, swerveSubsystem, swerveSubsystem::getPose, 2,
        // OffsetFromTargetAprTag.CENTER));
        //        new JoystickButton(driverJoystick, Constants.buttonB)
        //                .whileTrue(new GoToTagCommand(
        //                        photonCamera, swerveSubsystem, swerveSubsystem::getPose, 2,
        // OffsetFromTargetAprTag.LEFT));
        //        new JoystickButton(driverJoystick, Constants.buttonX)
        //                .whileTrue(new GoToTagCommand(
        //                        photonCamera, swerveSubsystem, swerveSubsystem::getPose, 2,
        // OffsetFromTargetAprTag.RIGHT));

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
                .onTrue(new HomeCommand(
                        armSubsystem, elevatorSubsystem, wristSubsystem, gripperSubsystem, bucketSubsystem));

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
        // new JoystickButton(coDriverJoystick, OIConstants.kDriverButtonB)
                // .onTrue(new GroundDropCommand(armSubsystem, elevatorSubsystem, wristSubsystem));

        // ground pickup
        // new JoystickButton(coDriverJoystick, OIConstants.kDriverLeftBumper)
                // .onTrue(new GroundPickupCommand(elevatorSubsystem, wristSubsystem, armSubsystem, gripperSubsystem));

        // shelf pickup
        // new JoystickButton(coDriverJoystick, OIConstants.kDriverRightBumper)
                // .onTrue(new ShelfPickupCommand(armSubsystem, elevatorSubsystem, wristSubsystem));

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

        new JoystickButton(coDriverJoystick, 6)
                .and(() -> armSubsystem.getArmPositionState() == ArmPosition.BUCKET_PICKUP)
                .whileTrue(new InstantCommand(armSubsystem::setZero).repeatedly())
                .onFalse(new BucketPickupCommand(
                        elevatorSubsystem, wristSubsystem, bucketSubsystem, armSubsystem, gripperSubsystem));
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
