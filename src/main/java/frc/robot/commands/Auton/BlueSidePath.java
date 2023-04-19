package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.ArmMoveCommands.MoveGripperCommand;
import frc.robot.commands.ArmPositionCommands.BucketPickupCommand;
import frc.robot.commands.ArmPositionCommands.GroundPickupCommand;
import frc.robot.commands.ArmPositionCommands.MidDropCommand;
import frc.robot.commands.DriveFollowPath;

public class BlueSidePath extends SequentialCommandGroup {
    public BlueSidePath() {
        addCommands(
                new DropPreloadedHigh("BlueSidePath_0"),
                new ParallelCommandGroup(
                        new DriveFollowPath("BlueSidePath_1", 1.5, 3, false),
                        new SequentialCommandGroup(
                                new WaitCommand(2),
                                new GroundPickupCommand(
                                        RobotContainer.getElevatorSubsystem(),
                                        RobotContainer.getWristSubsystem(),
                                        RobotContainer.getArmSubsystem(),
                                        RobotContainer.getGripperSubsystem()),
                                new WaitCommand(1.8),
                                new MoveGripperCommand(
                                        RobotContainer.getGripperSubsystem(),
                                        RobotContainer.getArmSubsystem(),
                                        MoveGripperCommand.GripperState.CLOSED),
                                new WaitCommand(0.2),
                                new InstantCommand(
                                        () -> RobotContainer.getWristSubsystem().setPositionOverride(true, 24.0)),
                                new MidDropCommand(
                                        RobotContainer.getArmSubsystem(),
                                        RobotContainer.getElevatorSubsystem(),
                                        RobotContainer.getWristSubsystem()))),
                new MoveGripperCommand(
                        RobotContainer.getGripperSubsystem(),
                        RobotContainer.getArmSubsystem(),
                        MoveGripperCommand.GripperState.OPENED),
                new WaitCommand(0.45),
                new ParallelCommandGroup(
                        new DriveFollowPath("BlueSidePath_2", 1, 3, false),
                        new InstantCommand(
                                        () -> RobotContainer.getWristSubsystem().setPositionOverride(false))
                                .andThen(new WaitCommand(0.2))
                                .andThen(new BucketPickupCommand(
                                        RobotContainer.getElevatorSubsystem(),
                                        RobotContainer.getWristSubsystem(),
                                        RobotContainer.getBucketSubsystem(),
                                        RobotContainer.getArmSubsystem(),
                                        RobotContainer.getGripperSubsystem()))));
    }
}
