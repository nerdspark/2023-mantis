// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DriveFollowPath;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.GoToTagCommand;
import frc.robot.commands.ArmMoveCommands.MoveGripperCommand;
import frc.robot.commands.ArmMoveCommands.MoveGripperCommand.GripperState;
import frc.robot.commands.ArmPositionCommands.GroundPickupCommand;
import frc.robot.commands.ArmPositionCommands.ScoreHighPositionCommand;
import frc.robot.commands.ArmPositionCommands.ScoreMidPositionCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

/** An example command that uses an example subsystem. */
public class ThreeElementWMarkers extends SequentialCommandGroup {

  public Command loadPathPlannerTrajectoryCommand(String filename, boolean resetOdometry){
          // 1. Load file and apply constraints
    PathPlannerTrajectory trajectory = PathPlanner.loadPath(filename, new PathConstraints(
      AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared));

      AutoConstants.autoEventMap.put("Marker1", new GroundPickupCommand(RobotContainer.getElevatorSubsystem(), RobotContainer.getWristSubsystem(), RobotContainer.getArmSubsystem(), RobotContainer.getGripperSubsystem()));
      AutoConstants.autoEventMap.put("Marker2", new GroundPickupCommand(RobotContainer.getElevatorSubsystem(), RobotContainer.getWristSubsystem(), RobotContainer.getArmSubsystem(), RobotContainer.getGripperSubsystem()));



          // 2. Define PID controllers for tracking trajectory
    PIDController xController = new PIDController(AutoConstants.kPXController, AutoConstants.kIXController, AutoConstants.kDXController);
    PIDController yController = new PIDController(AutoConstants.kPYController, AutoConstants.kIYController, AutoConstants.kIYController);
    ProfiledPIDController thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, AutoConstants.kIThetaController, AutoConstants.kDThetaController, AutoConstants.kThetaControllerConstraints);
    // PIDController thetaController = new PIDController(AutoConstants.kPThetaController, AutoConstants.kIThetaController, AutoConstants.kDThetaController);
    thetaController.enableContinuousInput(-Math.PI, Math.PI); // -Math.PI or -180?
  
    // 3. Construct command to follow trajectory with WPILIB trajectory follower
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            trajectory,
            RobotContainer.getSwerveSubsystem()::getPose,
            DriveConstants.kDriveKinematics,
            xController,
            yController,
            thetaController,
            RobotContainer.getSwerveSubsystem()::setModuleStates,
            RobotContainer.getSwerveSubsystem());

    // PPSwerveControllerCommand ppSwerveControllerCommand = new PPSwerveControllerCommand(
    //     trajectory,
    //     RobotContainer.getSwerveSubsystem()::getPose(),
    //     DriveConstants.kDriveKinematics,
    //     xController, 
    //     yController, 
    //     thetaController, 
    //     RobotContainer.getSwerveSubsystem()::setModuleStates,
    //     RobotContainer.getSwerveSubsystem());

    FollowPathWithEvents commandFPWE = new FollowPathWithEvents(swerveControllerCommand, 
      trajectory.getMarkers(), 
      AutoConstants.autoEventMap);

    // 4. Run path following command and stop at the end.
    if (resetOdometry) {
      return new SequentialCommandGroup(
          new InstantCommand(() -> RobotContainer.getSwerveSubsystem().resetOdometry(trajectory.getInitialHolonomicPose())),
          commandFPWE);
    } else {
      return swerveControllerCommand;
    }


  }

  public ThreeElementWMarkers(SwerveSubsystem swerveSubsystem){

    addCommands(
      new ScoreHighPositionCommand(RobotContainer.getArmSubsystem(), RobotContainer.getElevatorSubsystem(), RobotContainer.getWristSubsystem()),
      new MoveGripperCommand(RobotContainer.getGripperSubsystem(), RobotContainer.getArmSubsystem(), GripperState.Open),
      new ScoreMidPositionCommand(RobotContainer.getArmSubsystem(), RobotContainer.getElevatorSubsystem(), RobotContainer.getWristSubsystem()),
      new ParallelCommandGroup(

        loadPathPlannerTrajectoryCommand("threeElementRed_1", true)
      //   new GroundPickupCommand(RobotContainer.getElevatorSubsystem(), RobotContainer.getWristSubsystem(), RobotContainer.getArmSubsystem(), RobotContainer.getGripperSubsystem())
       ),

      new MoveGripperCommand(RobotContainer.getGripperSubsystem(), RobotContainer.getArmSubsystem(), GripperState.Closed),
      new ParallelCommandGroup(
        loadPathPlannerTrajectoryCommand("threeElementRed_2", false),
        new ScoreMidPositionCommand(RobotContainer.getArmSubsystem(), RobotContainer.getElevatorSubsystem(), RobotContainer.getWristSubsystem())
      ),
      new ScoreHighPositionCommand(RobotContainer.getArmSubsystem(), RobotContainer.getElevatorSubsystem(), RobotContainer.getWristSubsystem()),
      new MoveGripperCommand(RobotContainer.getGripperSubsystem(), RobotContainer.getArmSubsystem(), GripperState.Open),
      new ParallelCommandGroup(

        loadPathPlannerTrajectoryCommand("threeElementRed_3", false)
      //   new GroundPickupCommand(RobotContainer.getElevatorSubsystem(), RobotContainer.getWristSubsystem(), RobotContainer.getArmSubsystem(), RobotContainer.getGripperSubsystem())
      ),
      new ScoreHighPositionCommand(RobotContainer.getArmSubsystem(), RobotContainer.getElevatorSubsystem(), RobotContainer.getWristSubsystem()),
      new MoveGripperCommand(RobotContainer.getGripperSubsystem(), RobotContainer.getArmSubsystem(), GripperState.Closed),
      new ParallelCommandGroup(
        loadPathPlannerTrajectoryCommand("threeElementRed_4", false),
        new ScoreMidPositionCommand(RobotContainer.getArmSubsystem(), RobotContainer.getElevatorSubsystem(), RobotContainer.getWristSubsystem())
      )
    );

    // addCommands(
    //   loadPathPlannerTrajectoryCommand("threeConePathOne", true),
    //   loadPathPlannerTrajectoryCommand("threeConePathTwo", false),
    //   loadPathPlannerTrajectoryCommand("threeConePathThree", false),
    //   loadPathPlannerTrajectoryCommand("threeConePathFour", false)
    // );
  }
}
