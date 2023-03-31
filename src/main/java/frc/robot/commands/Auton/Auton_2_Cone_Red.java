// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.ArmMoveCommands.MoveGripperCommand;
import frc.robot.commands.ArmMoveCommands.MoveGripperCommand.GripperState;
import frc.robot.commands.ArmPositionCommands.GroundPickupCommand;
import frc.robot.commands.ArmPositionCommands.HighDropCommand;
import frc.robot.commands.ArmPositionCommands.MidDropCommand;
import frc.robot.commands.DriveFollowPath;
import frc.robot.subsystems.SwerveSubsystem;

/** An example command that uses an example subsystem. */
public class Auton_2_Cone_Red extends SequentialCommandGroup {

    public Command loadPathPlannerTrajectoryCommand(String filename) {
        // 1. Load file and apply constraints
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("threeElementRed_1", 1.0, 0.5);

        AutoConstants.autoEventMap.put(
                "Marker1",
                new GroundPickupCommand(
                        RobotContainer.getElevatorSubsystem(),
                        RobotContainer.getWristSubsystem(),
                        RobotContainer.getArmSubsystem(),
                        RobotContainer.getGripperSubsystem()));

        // 3. Construct command to follow trajectory with WPILIB trajectory

        DriveFollowPath driveFollowPathCmd = new DriveFollowPath("threeElementRed_2", 1, 0.5, true);

        // PPSwerveControllerCommand ppSwerveControllerCommand = new PPSwerveControllerCommand(
        //     trajectory,
        //     RobotContainer.getSwerveSubsystem()::getPose(),
        //     DriveConstants.kDriveKinematics,
        //     xController,
        //     yController,
        //     thetaController,
        //     RobotContainer.getSwerveSubsystem()::setModuleStates,
        //     RobotContainer.getSwerveSubsystem());

        FollowPathWithEvents commandFPWE =
                new FollowPathWithEvents(driveFollowPathCmd, trajectory.getMarkers(), AutoConstants.autoEventMap);

        return commandFPWE;
    }

    public Auton_2_Cone_Red(SwerveSubsystem swerveSubsystem) {

        addCommands(
                new HighDropCommand(
                        RobotContainer.getArmSubsystem(),
                        RobotContainer.getElevatorSubsystem(),
                        RobotContainer.getWristSubsystem()),
                new MoveGripperCommand(
                        RobotContainer.getGripperSubsystem(), RobotContainer.getArmSubsystem(), GripperState.OPENED),
                new WaitCommand(0.2),
                new MidDropCommand(
                        RobotContainer.getArmSubsystem(),
                        RobotContainer.getElevatorSubsystem(),
                        RobotContainer.getWristSubsystem()),
                loadPathPlannerTrajectoryCommand("threeElementRed_1"));
    }
}
