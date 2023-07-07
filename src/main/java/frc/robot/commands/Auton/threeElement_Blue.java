// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import frc.robot.commands.ArmMoveCommands.MoveGripperCommand;
import frc.robot.commands.ArmMoveCommands.MoveGripperCommand.GripperState;
import frc.robot.commands.ArmPositionCommands.BucketPickupCommand;
import frc.robot.commands.ArmPositionCommands.GroundPickupCommand;
import frc.robot.commands.ArmPositionCommands.HighDropCommand;
import frc.robot.commands.ArmPositionCommands.MidDropCommand;
import frc.robot.commands.DriveFollowPath;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class threeElement_Blue extends SequentialCommandGroup {
    public threeElement_Blue(SwerveSubsystem swerveSubsystem, LimelightSubsystem limelightSubsystem) {
        addCommands(
                new InstantCommand(() -> swerveSubsystem.setRobotAngleOffset(0)),
                new DropPreloadedHigh("BlueFluurb_0"),
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
                new InstantCommand(() -> RobotContainer.getArmSubsystem().setPosition(87)),
                new RepeatCommand(new SwerveJoystickCmd(
                                        swerveSubsystem,
                                        limelightSubsystem,
                                        () -> -0.3,
                                        () -> 0.0,
                                        () -> 0.0,
                                        () -> 0.0,
                                        () -> false,
                                        () -> 0,
                                        () -> 0.0,
                                        () -> 0.0,
                                        () -> false,
                                        () -> false,
                                        () -> false,
                                        () -> false)
                                .repeatedly())
                        .raceWith(new WaitCommand(0.6)),
                new HighDropCommand(
                        RobotContainer.getArmSubsystem(),
                        RobotContainer.getElevatorSubsystem(),
                        RobotContainer.getWristSubsystem()),
                new InstantCommand(() -> RobotContainer.getArmSubsystem().setPosition(84)),
                new WaitCommand(0.1),
                new InstantCommand(() -> {
                    RobotContainer.getGripperSubsystem().setLeftPosition(-15);
                    RobotContainer.getGripperSubsystem().setRightPosition(2);
                }),
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
