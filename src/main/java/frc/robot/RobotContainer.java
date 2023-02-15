// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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

    private static final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
      
    SendableChooser<Command> chooser = new SendableChooser<>();

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


    // chooser.addOption("line4mTwist", loadPathplannerTrajectoryToSwerveController(
    //   "line4mTwist",
    //   true));
    chooser.addOption("line2meters", loadPathplannerTrajectoryToSwerveController(
      "line2meters",
      true));
    // chooser.addOption("ShortL2", loadPathplannerTrajectoryToSwerveController(
    //   "ShortL2",
    //   true));
    // chooser.addOption("square", loadPathplannerTrajectoryToSwerveController(
    //   "square",
    //   true));
    // chooser.addOption("L_withTwist", loadPathplannerTrajectoryToSwerveController(
    //   "L_withTwist",
    //   true));
    chooser.addOption("ShortL2_withTwist", loadPathplannerTrajectoryToSwerveController(
      "ShortL2_withTwist",
      true));
    // chooser.addOption("ShortR2_withTwist", loadPathplannerTrajectoryToSwerveController(
    //   "ShortR2_withTwist",
    //   true));
    // chooser.addOption("ShortR2", loadPathplannerTrajectoryToSwerveController(
    //   "ShortR2",
    //   true));
    // chooser.addOption("straight_twist", loadPathplannerTrajectoryToSwerveController(
    //   "straight_twist",
    //   true));
    // chooser.addOption("DriveRectangle", loadPathplannerTrajectoryToSwerveController(
    //   "DriveRectangle",
    //   true));
    // chooser.addOption("ShortR2_withTwist", loadPathplannerTrajectoryToSwerveController(
    //   "pathplanner/generatedJSON/ShortR2_withTwist.wpilib.json",
    //   true));

    //Pathgroup
    chooser.addOption("threeCone", followTrajectoryCommand(
      PathPlanner.loadPath("threeConePathOne", new PathConstraints(3, 2)), 
      PathPlanner.loadPath("threeConePathTwo", new PathConstraints(3, 2)), 
      PathPlanner.loadPath("threeConePathThree", new PathConstraints(3, 2)),
      PathPlanner.loadPath("threeConePathFour", new PathConstraints(3, 2),false), true));

    //Pathgroup with markers
    // chooser.addOption("DriveRectanglePathGroupWithEvents", followTrajectoryCommandMarker(
    //   "DriveRectangleWithEvents"));  

    Shuffleboard.getTab("Autonomous").add(chooser);
  }


  //Full auto command...includes markers

// public Command followTrajectoryCommandMarker(String filename){

//   // This will load the file "FullAuto.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
// // for every path in the group
// List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("DriveRectanglePathGroupWithEvents",
//  new PathConstraints(1, 0.5),
//  new PathConstraints(2, 1));

// // This is just an example event map. It would be better to have a constant, global event map
// // in your code that will be used by all path following commands.
// HashMap<String, Command> eventMap = new HashMap<>();
// eventMap.put("Vision1", new PrintCommand("Passed marker 1"));
// eventMap.put("PositionArm2", new PrintCommand("Passed marker 2"));

// // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.

// SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
//     swerveSubsystem::getPose, // Pose2d supplier
//     swerveSubsystem.resetOdometry(pathGroup.getInitialHolonomicPose), // Pose2d consumer, used to reset odometry at the beginning of auto
//     DriveConstants.kDriveKinematics, // SwerveDriveKinematics
//     new PIDController(2.5, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
//     new PIDController(2.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
//     swerveSubsystem::setModuleStates, // Module states consumer used to output to the drive subsystem
//     eventMap,
//     true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
//     swerveSubsystem // The drive subsystem. Used to properly set the requirements of path following commands
// );

// Command driveRectanglePathGroupWithEvents = autoBuilder.driveRectanglePathGroupWithEvents(pathGroup);
// // Command fullAuto = autoBuilder.fullAuto(pathGroup);


//   return new SequentialCommandGroup(
//     new FollowPathWithEvents(autoBuilder(pathGroup), null, eventMap)
//   )

    // new FollowPathWithEvents(
    //   new autoBuilder(pathGroup.get(0)), pathGroup.getMarkers(0), eventMap),
    // new WaitCommand(3),
    // ;

  //  new FollowPathWithEvents(
  //     new autoBuilder(pathGroup.get(0), swerveSubsystem, true),
  //     pathGroup.get(0).getMarkers(),
  //     eventMap),
  //   new WaitCommand(3),
  //   new FollowPathWithEvents(
  //     new FollowPath(pathGroup.get(1), swerveSubsystem, false),
  //     pathGroup.get(1).getMarkers(),
  //     eventMap));


// }

//Follow Pathgroup Command.


  public Command followTrajectoryCommand(PathPlannerTrajectory traj, PathPlannerTrajectory traj2, PathPlannerTrajectory traj3,
  PathPlannerTrajectory traj4, boolean isFirstPath) {

    return new SequentialCommandGroup(
         new InstantCommand(() -> {
           // Reset odometry for the first path you run during auto
           if(isFirstPath){
               swerveSubsystem.resetOdometry(traj.getInitialHolonomicPose());
           }
         }),
         new PPSwerveControllerCommand(
             traj, 
             swerveSubsystem::getPose, // Pose supplier
             DriveConstants.kDriveKinematics, // SwerveDriveKinematics
             new PIDController(2.5, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
             new PIDController(2.5, 0, 0), // Y controller (usually the same values as X controller)
             new PIDController(1.0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
             swerveSubsystem::setModuleStates, // Module states consumer
             false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
             swerveSubsystem // Requires this drive subsystem
         ),
        //  new WaitCommand(1),
         new PPSwerveControllerCommand(
            traj2, 
            swerveSubsystem::getPose, // Pose supplier
            DriveConstants.kDriveKinematics, // SwerveDriveKinematics
            new PIDController(2.5, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            new PIDController(2.5, 0, 0), // Y controller (usually the same values as X controller)
            new PIDController(1.0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            swerveSubsystem::setModuleStates, // Module states consumer
            false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            swerveSubsystem // Requires this drive subsystem
          ),
          // new WaitCommand(1),
          new PPSwerveControllerCommand(
            traj3, 
            swerveSubsystem::getPose, // Pose supplier
            DriveConstants.kDriveKinematics, // SwerveDriveKinematics
            new PIDController(2.5, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            new PIDController(2.5, 0, 0), // Y controller (usually the same values as X controller)
            new PIDController(1.0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            swerveSubsystem::setModuleStates, // Module states consumer
            false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            swerveSubsystem // Requires this drive subsystem
          ),
          // new WaitCommand(1),
          new PPSwerveControllerCommand(
            traj4, 
            swerveSubsystem::getPose, // Pose supplier
            DriveConstants.kDriveKinematics, // SwerveDriveKinematics
            new PIDController(2.5, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            new PIDController(2.5, 0, 0), // Y controller (usually the same values as X controller)
            new PIDController(1.0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            swerveSubsystem::setModuleStates, // Module states consumer
            false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            swerveSubsystem // Requires this drive subsystem
          ));
 }

  public Command loadPathplannerTrajectoryToSwerveController(String filename, boolean resetOdomtry) { 
    // Trajectory trajectory;
    PathPlannerTrajectory trajectory = PathPlanner.loadPath(filename, new PathConstraints(1, 0.5));

    // try {
    //   Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filename);
    //   trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    // } catch (IOException exception) {
    //   DriverStation.reportError("Unable to open trajectory" + filename, exception.getStackTrace());
    //   System.out.println("Unable to read from file " + filename);
    //   return new InstantCommand();
    // }

    // 3. Define PID controllers for tracking trajectory
    PIDController xController = new PIDController(AutoConstants.kPXController, AutoConstants.kIXController, AutoConstants.kDXController);
    PIDController yController = new PIDController(AutoConstants.kPYController, AutoConstants.kIYController, AutoConstants.kIYController);
    ProfiledPIDController thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, AutoConstants.kIThetaController, AutoConstants.kDThetaController, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI); // -Math.PI or -180?
  
    // 4. Construct command to follow trajectory with WPILIB trajectory follower
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            trajectory,
            swerveSubsystem::getPose,
            DriveConstants.kDriveKinematics,
            xController,
            yController,
            thetaController,
            swerveSubsystem::setModuleStates,
            swerveSubsystem);

    if (resetOdomtry) {
      return new SequentialCommandGroup(
          new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
          swerveControllerCommand);
    } else {
      return swerveControllerCommand;
    }

  }
    // // 5. Construct command to follow trajectory using the Path Planner trajectory follower

  //   PIDController xController = new PIDController(AutoConstants.kPXController, AutoConstants.kIXController, AutoConstants.kDXController);
  //   PIDController yController = new PIDController(AutoConstants.kPYController, AutoConstants.kIYController, AutoConstants.kIYController);
  //   PIDController thetaController = new PIDController(AutoConstants.kPThetaController, AutoConstants.kIThetaController, AutoConstants.kDThetaController);
  //   thetaController.enableContinuousInput(-Math.PI, Math.PI); // -Math.PI or -180?

  //   PPSwerveControllerCommand ppSwerveControllerCommand = new PPSwerveControllerCommand(
  //         trajectory, 
  //         swerveSubsystem::getPose, // Pose supplier
  //         DriveConstants.kDriveKinematics,// SwerveDriveKinematics
  //         xController, // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
  //         yController, // Y controller (usually the same values as X controller)
  //         thetaController, // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
  //         swerveSubsystem::setModuleStates, // Module states consumer
  //         false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
  //         swerveSubsystem // Requires this drive subsystem
  //       );

  //   return new SequentialCommandGroup(
  //         new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())), 
  //         ppSwerveControllerCommand, 
  //         new InstantCommand(() -> swerveSubsystem.stopModules()));


  //   // )
        
    
  //   // if (resetOdomtry) {
  //   //   return new SequentialCommandGroup(
  //   //       new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())), ppSwerveControllerCommand,
  //   //       new InstantCommand(() -> swerveSubsystem.stopModules()));
  //   // } else {
  //   //   return ppSwerveControllerCommand;
  //   // }

  // }






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
    // new JoystickButton(cont, Constants.buttonA).whileHeld(new ExampleCommand(m_exampleSubsystem));

    // // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return chooser.getSelected();

    // // 7. Test 0 to Autonomous code...seems like old Y axis...Right is +  
    // // 1. Create trajectory settings
    //   TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
    //           AutoConstants.kMaxSpeedMetersPerSecond,
    //           AutoConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(DriveConstants.kDriveKinematics);

    //   // 2. Generate trajectory
    //   Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
    //           new Pose2d(0, 0, new Rotation2d(0)),
    //           List.of(
    //                   new Translation2d(1, 0),
    //                   new Translation2d(1, -1)),
    //           new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
    //           trajectoryConfig);

    //   // 3. Define PID controllers for tracking trajectory
    //   PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    //   PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
    //   ProfiledPIDController thetaController = new ProfiledPIDController(
    //           AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    //   thetaController.enableContinuousInput(-Math.PI, Math.PI);

    //   // 4. Construct command to follow trajectory
    //   SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
    //           trajectory,
    //           swerveSubsystem::getPose,
    //           DriveConstants.kDriveKinematics,
    //           xController,
    //           yController,
    //           thetaController,
    //           swerveSubsystem::setModuleStates,
    //           swerveSubsystem);

    //   // 5. Add some init and wrap-up, and return everything
      // return new SequentialCommandGroup(
      //         new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
      //         swerveControllerCommand,
      //         new InstantCommand(() -> swerveSubsystem.stopModules()));

  }
}
