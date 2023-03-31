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
public class threeElement_Red extends SequentialCommandGroup {

    // public line2metersCommand(SwerveSubsystem swervesubsystem) {
    // }

    public threeElement_Red(SwerveSubsystem swerveSubsystem) {
        addCommands(
                new DriveFollowPath("threeElementRed_L0", 1, 0.5, true),
                new MoveGripperCommand(
                        RobotContainer.getGripperSubsystem(), RobotContainer.getArmSubsystem(), GripperState.CLOSED),
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
                new ParallelCommandGroup(
                        new DriveFollowPath("threeElementRed_L1", 3, 2, true),
                        new SequentialCommandGroup(
                                new WaitCommand(0.5),
                                new GroundPickupCommand(
                                        RobotContainer.getElevatorSubsystem(),
                                        RobotContainer.getWristSubsystem(),
                                        RobotContainer.getArmSubsystem(),
                                        RobotContainer.getGripperSubsystem()),
                                //                                new WaitUntilCommand(() ->
                                // (RobotContainer.getTimeOfFlightSubsystem()
                                //                                                                .getRange()
                                //                                                        < 75)
                                //                                                &&
                                // (RobotContainer.getTimeOfFlightSubsystem()
                                //                                                                .getRange()
                                //                                                        > 32))
                                //                                        .andThen(() -> System.out.println("**********
                                // TOF sensor <75 "
                                //                                                +
                                // RobotContainer.getTimeOfFlightSubsystem()
                                //                                                        .getRange()))
                                //                                        .withTimeout(2.5),
                                new WaitCommand(1.85),
                                new MoveGripperCommand(
                                        RobotContainer.getGripperSubsystem(),
                                        RobotContainer.getArmSubsystem(),
                                        GripperState.CLOSED),
                                new InstantCommand(
                                        () -> RobotContainer.getWristSubsystem().setPositionOverride(true, 22.0)),
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
                new MoveGripperCommand(
                        RobotContainer.getGripperSubsystem(), RobotContainer.getArmSubsystem(), GripperState.OPENED),
                new WaitCommand(0.5),
                new InstantCommand(() -> RobotContainer.getWristSubsystem().setPositionOverride(false)),
                new InstantCommand(() -> RobotContainer.getGripperSubsystem().setSwap(false)),
                new HomeCommand(
                        RobotContainer.getArmSubsystem(),
                        RobotContainer.getElevatorSubsystem(),
                        RobotContainer.getWristSubsystem(),
                        RobotContainer.getGripperSubsystem(),
                        RobotContainer.getBucketSubsystem())
                //                ,
                //                new ParallelCommandGroup(
                //                        new DriveFollowPath("threeElementRed_2", 1, 0.5, false),
                //
                //                new HighDropCommand(
                //                        RobotContainer.getArmSubsystem(),
                //                        RobotContainer.getElevatorSubsystem(),
                //                        RobotContainer.getWristSubsystem()),
                //                new MoveGripperCommand(
                //                        RobotContainer.getGripperSubsystem(), RobotContainer.getArmSubsystem(),
                // GripperState.OPENED),
                //                new WaitCommand(0.2),
                //                new InstantCommand(() ->
                // RobotContainer.getWristSubsystem().setPositionOverride(false)),
                //                new InstantCommand(() -> RobotContainer.getGripperSubsystem().setSwap(false)),
                //                new ParallelCommandGroup(
                //                                                new DriveFollowPath("threeElementRed_3", 1, 0.5,
                // false),
                //                        new SequentialCommandGroup(
                //                                new WaitCommand(1),
                //                                new MidDropCommand(
                //                                        RobotContainer.getArmSubsystem(),
                //                                        RobotContainer.getElevatorSubsystem(),
                //                                        RobotContainer.getWristSubsystem()))),
                //                new GroundPickupCommand(
                //                        RobotContainer.getElevatorSubsystem(),
                //                        RobotContainer.getWristSubsystem(),
                //                        RobotContainer.getArmSubsystem(),
                //                        RobotContainer.getGripperSubsystem()),
                //                new InstantCommand(() -> RobotContainer.getWristSubsystem().setPositionOverride(true,
                // 22.0)),
                //                new InstantCommand(() -> RobotContainer.getGripperSubsystem().setSwap(true)),
                //                new WaitUntilCommand(() -> (RobotContainer.getTimeOfFlightSubsystem().getRange() <
                // 140) && (RobotContainer.getTimeOfFlightSubsystem().getRange() > 0)),
                //                new MoveGripperCommand(
                //                        RobotContainer.getGripperSubsystem(), RobotContainer.getArmSubsystem(),
                // GripperState.CLOSED),
                //                new WaitCommand(0.2),
                //                new ParallelCommandGroup(
                //                         new DriveFollowPath("threeElementRed_4", 1, 0.5, false),
                //                        new MidDropCommand(
                //                                RobotContainer.getArmSubsystem(),
                //                                RobotContainer.getElevatorSubsystem(),
                //                                RobotContainer.getWristSubsystem())),
                //                new InstantCommand(() -> RobotContainer.getGripperSubsystem().setSwap(false)),
                //                new InstantCommand(() ->
                // RobotContainer.getWristSubsystem().setPositionOverride(false))
                );
    }
}
