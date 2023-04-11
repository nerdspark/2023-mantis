package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.ArmMoveCommands.MoveArmCommand;
import frc.robot.commands.ArmMoveCommands.MoveGripperCommand;
import frc.robot.commands.ArmMoveCommands.MoveWristCommand;
import frc.robot.commands.ArmPositionCommands.*;
import frc.robot.commands.DriveFollowPath;

public class RedSidePath extends SequentialCommandGroup {
    public RedSidePath() {
        addCommands(
                new MoveGripperCommand(
                        RobotContainer.getGripperSubsystem(),
                        RobotContainer.getArmSubsystem(),
                        MoveGripperCommand.GripperState.CLOSED),
                new ParallelCommandGroup(
                        new DriveFollowPath("RedSidePath_0", 1, 0.5, true),
                        new HighDropCommand(
                                RobotContainer.getArmSubsystem(),
                                RobotContainer.getElevatorSubsystem(),
                                RobotContainer.getWristSubsystem())),
                new WaitCommand(0.3),
                new MoveArmCommand(
                        RobotContainer.getArmSubsystem(),
                        87.0,
                        Constants.ArmConstants.highDropPosition.smartMotionMaxVel(),
                        Constants.ArmConstants.highDropPosition.smartMotionMaxAccel()),
                new MoveGripperCommand(
                        RobotContainer.getGripperSubsystem(),
                        RobotContainer.getArmSubsystem(),
                        MoveGripperCommand.GripperState.OPENED),
                new WaitCommand(0.2),
                new MidDropCommand(
                        RobotContainer.getArmSubsystem(),
                        RobotContainer.getElevatorSubsystem(),
                        RobotContainer.getWristSubsystem()),
                new ParallelCommandGroup(
                        new DriveFollowPath("RedSidePath_1", 1.2, 1.5, false),
                        new SequentialCommandGroup(
                                new WaitCommand(1),
                                new GroundPickupCommand(
                                        RobotContainer.getElevatorSubsystem(),
                                        RobotContainer.getWristSubsystem(),
                                        RobotContainer.getArmSubsystem(),
                                        RobotContainer.getGripperSubsystem()),
                                new WaitCommand(3.25),
                                new MoveGripperCommand(
                                        RobotContainer.getGripperSubsystem(),
                                        RobotContainer.getArmSubsystem(),
                                        MoveGripperCommand.GripperState.CLOSED),
                                new WaitCommand(0.2))),
                new InstantCommand(() -> RobotContainer.getWristSubsystem().setPositionOverride(true, 24.0)),
                new ParallelCommandGroup(
                        new DriveFollowPath("RedSidePath_2", 1.2, 2.5, false),
                        new WaitCommand(0.5)
                                .andThen(new MidDropCommand(
                                        RobotContainer.getArmSubsystem(),
                                        RobotContainer.getElevatorSubsystem(),
                                        RobotContainer.getWristSubsystem()))),
                new InstantCommand(() -> RobotContainer.getWristSubsystem().setPositionOverride(false)),
                new MoveGripperCommand(
                        RobotContainer.getGripperSubsystem(),
                        RobotContainer.getArmSubsystem(),
                        MoveGripperCommand.GripperState.OPENED),
                new InstantCommand(() -> RobotContainer.getGripperSubsystem().setSwap(false)),
                new ParallelCommandGroup(
                        new DriveFollowPath("RedSidePath_3", 2, 3, false),
                        new WaitCommand(0.3)
                                .andThen(new MoveWristCommand(
                                                RobotContainer.getWristSubsystem(),
                                                Constants.ArmConstants.midDropPosition.wristCmdPos())
                                        .andThen(new BucketPickupCommand(
                                                RobotContainer.getElevatorSubsystem(),
                                                RobotContainer.getWristSubsystem(),
                                                RobotContainer.getBucketSubsystem(),
                                                RobotContainer.getArmSubsystem(),
                                                RobotContainer.getGripperSubsystem())))));
    }
}
