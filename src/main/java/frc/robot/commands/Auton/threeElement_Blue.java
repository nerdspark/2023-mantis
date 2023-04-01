// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import frc.robot.commands.ArmMoveCommands.MoveGripperCommand;
import frc.robot.commands.ArmMoveCommands.MoveGripperCommand.GripperState;
import frc.robot.commands.ArmPositionCommands.*;
import frc.robot.commands.DriveFollowPath;
import frc.robot.subsystems.*;

/** An example command that uses an example subsystem. */
public class threeElement_Blue extends SequentialCommandGroup {
    public threeElement_Blue(SwerveSubsystem swerveSubsystem) {
        addCommands(
                new ParallelCommandGroup(
                        new DriveFollowPath("threeElementBlue_L0", 1, 0.5, true),
                        new MoveGripperCommand(
                                RobotContainer.getGripperSubsystem(),
                                RobotContainer.getArmSubsystem(),
                                GripperState.CLOSED),
                        new HighDropCommand(
                                RobotContainer.getArmSubsystem(),
                                RobotContainer.getElevatorSubsystem(),
                                RobotContainer.getWristSubsystem())),
                new WaitCommand(0.1),
                new MoveGripperCommand(
                        RobotContainer.getGripperSubsystem(), RobotContainer.getArmSubsystem(), GripperState.OPENED),
                new WaitCommand(0.2),
                new MidDropCommand(
                        RobotContainer.getArmSubsystem(),
                        RobotContainer.getElevatorSubsystem(),
                        RobotContainer.getWristSubsystem()),
                new ParallelCommandGroup(
                        new DriveFollowPath("threeElementBlue_L1", 3, 2, false),
                        new SequentialCommandGroup(
                                new WaitCommand(0.4),
                                new GroundPickupCommand(
                                        RobotContainer.getElevatorSubsystem(),
                                        RobotContainer.getWristSubsystem(),
                                        RobotContainer.getArmSubsystem(),
                                        RobotContainer.getGripperSubsystem()),
                                new WaitCommand(1.75),
                                new MoveGripperCommand(
                                        RobotContainer.getGripperSubsystem(),
                                        RobotContainer.getArmSubsystem(),
                                        GripperState.CLOSED),
                                new InstantCommand(
                                        () -> RobotContainer.getWristSubsystem().setPositionOverride(true, 24.0)),
                                new InstantCommand(() ->
                                        RobotContainer.getGripperSubsystem().setSwap(true)),
                                new WaitCommand(0.2),
                                new MidDropCommand(
                                        RobotContainer.getArmSubsystem(),
                                        RobotContainer.getElevatorSubsystem(),
                                        RobotContainer.getWristSubsystem()))),
                new HighDropCommand(
                        RobotContainer.getArmSubsystem(),
                        RobotContainer.getElevatorSubsystem(),
                        RobotContainer.getWristSubsystem()),
                new WaitCommand(0.1),
                new MoveGripperCommand(
                        RobotContainer.getGripperSubsystem(), RobotContainer.getArmSubsystem(), GripperState.OPENED),
                new WaitCommand(0.6),
                new InstantCommand(() -> RobotContainer.getWristSubsystem().setPositionOverride(false)),
                new InstantCommand(() -> RobotContainer.getGripperSubsystem().setSwap(false)),
                new ParallelCommandGroup(
                        new HomeCommand(
                                RobotContainer.getArmSubsystem(),
                                RobotContainer.getElevatorSubsystem(),
                                RobotContainer.getWristSubsystem(),
                                RobotContainer.getGripperSubsystem(),
                                RobotContainer.getBucketSubsystem()),
                        new DriveFollowPath("threeElementBlue_L2", 5, 4, false)));
    }
}
