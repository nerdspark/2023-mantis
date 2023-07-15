package frc.robot.commands.Auton;

import static frc.robot.RobotContainer.swerveSubsystem;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.ArmMoveCommands.MoveGripperCommand;
import frc.robot.commands.ArmPositionCommands.BucketPickupCommand;
import frc.robot.commands.ArmPositionCommands.GroundPickupCommand;
import frc.robot.commands.DriveFollowPath;

public class RedBobFourCone extends SequentialCommandGroup {
    public RedBobFourCone() {
        addCommands(
                new InstantCommand(() -> swerveSubsystem.setRobotAngleOffset(Math.PI)),
                new InstantCommand(() -> RobotContainer.getGripperSubsystem().setLeftPosition(-10)),
                new ParallelCommandGroup(
                        new DriveFollowPath("RedBobFourCone_1", 2.5, 2, true),
                        new SequentialCommandGroup(
                                new WaitCommand(0.25),
                                new GroundPickupCommand(
                                        RobotContainer.getElevatorSubsystem(),
                                        RobotContainer.getWristSubsystem(),
                                        RobotContainer.getArmSubsystem(),
                                        RobotContainer.getGripperSubsystem()),
                                new WaitCommand(1.1),
                                new MoveGripperCommand(
                                        RobotContainer.getGripperSubsystem(),
                                        RobotContainer.getArmSubsystem(),
                                        MoveGripperCommand.GripperState.CLOSED),
                                new WaitCommand(0.5),
                                new InstantCommand(
                                        () -> RobotContainer.getWristSubsystem().setPositionOverride(true, 24.0)),
                                new InstantCommand(() ->
                                        RobotContainer.getGripperSubsystem().setSwap(true)),
                                new InstantCommand(
                                        () -> RobotContainer.getArmSubsystem().setPosition(130)))),
                new InstantCommand(() -> RobotContainer.getWristSubsystem().setPositionOverride(false)),
                new InstantCommand(() -> RobotContainer.getGripperSubsystem().setSwap(false)),
                new MoveGripperCommand(
                        RobotContainer.getGripperSubsystem(),
                        RobotContainer.getArmSubsystem(),
                        MoveGripperCommand.GripperState.OPENED),
                new ParallelCommandGroup(
                        new DriveFollowPath("RedBobFourCone_2", 3, 2, false),
                        new SequentialCommandGroup(
                                new WaitCommand(0.6),
                                new GroundPickupCommand(
                                        RobotContainer.getElevatorSubsystem(),
                                        RobotContainer.getWristSubsystem(),
                                        RobotContainer.getArmSubsystem(),
                                        RobotContainer.getGripperSubsystem()),
                                new WaitCommand(1.7),
                                new MoveGripperCommand(
                                        RobotContainer.getGripperSubsystem(),
                                        RobotContainer.getArmSubsystem(),
                                        MoveGripperCommand.GripperState.CLOSED),
                                new WaitCommand(0.5),
                                new InstantCommand(
                                        () -> RobotContainer.getWristSubsystem().setPositionOverride(true, 24.0)),
                                new InstantCommand(
                                        () -> RobotContainer.getArmSubsystem().setPosition(130)),
                                new WaitCommand(2.8),
                                new MoveGripperCommand(
                                        RobotContainer.getGripperSubsystem(),
                                        RobotContainer.getArmSubsystem(),
                                        MoveGripperCommand.GripperState.OPENED))),
                new InstantCommand(() -> RobotContainer.getWristSubsystem().setPositionOverride(false)),
                new ParallelCommandGroup(
                        new DriveFollowPath("RedBobFourCone_3", 3, 2, false),
                        new WaitCommand(0.2)
                                .andThen(new BucketPickupCommand(
                                        RobotContainer.getElevatorSubsystem(),
                                        RobotContainer.getWristSubsystem(),
                                        RobotContainer.getBucketSubsystem(),
                                        RobotContainer.getArmSubsystem(),
                                        RobotContainer.getGripperSubsystem()))));
    }
}
