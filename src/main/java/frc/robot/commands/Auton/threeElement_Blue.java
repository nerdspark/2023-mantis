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

public class threeElement_Blue extends SequentialCommandGroup {
    public threeElement_Blue(SwerveSubsystem swerveSubsystem) {
        addCommands(
                new MoveGripperCommand(
                        RobotContainer.getGripperSubsystem(),
                        RobotContainer.getArmSubsystem(),
                        GripperState.CLOSED),
                new ParallelCommandGroup(
                        new DriveFollowPath("BlueFluurb_0", 1, 0.5, true),
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
                        new DriveFollowPath("BlueFluurb_1", 2.75, 2, false),
                        new SequentialCommandGroup(
                                new WaitCommand(0.4),
                                new GroundPickupCommand(
                                        RobotContainer.getElevatorSubsystem(),
                                        RobotContainer.getWristSubsystem(),
                                        RobotContainer.getArmSubsystem(),
                                        RobotContainer.getGripperSubsystem()),
                                new WaitCommand(2.1),
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
                        new BucketPickupCommand(
                                RobotContainer.getElevatorSubsystem(),
                                RobotContainer.getWristSubsystem(),
                                RobotContainer.getBucketSubsystem(),
                                RobotContainer.getArmSubsystem(),
                                RobotContainer.getGripperSubsystem()),
                        new WaitCommand(0.25).andThen(new DriveFollowPath("BlueFluurb_2", 3, 1.75, false, true))));
    }
}
