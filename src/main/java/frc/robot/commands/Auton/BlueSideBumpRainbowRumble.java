package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.ArmMoveCommands.MoveArmCommand;
import frc.robot.commands.ArmMoveCommands.MoveGripperCommand;
import frc.robot.commands.ArmMoveCommands.MoveWristCommand;
import frc.robot.commands.ArmPositionCommands.BucketPickupCommand;
import frc.robot.commands.ArmPositionCommands.GroundPickupCommand;
import frc.robot.commands.ArmPositionCommands.HighDropCommand;
import frc.robot.commands.ArmPositionCommands.MidDropCommand;
import frc.robot.commands.DriveFollowPath;

public class BlueSideBumpRainbowRumble extends SequentialCommandGroup {
    public BlueSideBumpRainbowRumble() {
        addCommands(
                new MoveGripperCommand(
                        RobotContainer.getGripperSubsystem(),
                        RobotContainer.getArmSubsystem(),
                        MoveGripperCommand.GripperState.CLOSED),
                new ParallelCommandGroup(
                        new DriveFollowPath("BlueSidePath_0", 1, 0.5, true),
                        new HighDropCommand(
                                RobotContainer.getArmSubsystem(),
                                RobotContainer.getElevatorSubsystem(),
                                RobotContainer.getWristSubsystem())),
                new WaitCommand(0.1),
                new SequentialCommandGroup(
                        new MoveArmCommand(
                                RobotContainer.armSubsystem,
                                (ArmConstants.highDropPosition.armCmdPos() + 10),
                                ArmConstants.highDropPosition.smartMotionMaxVel(),
                                ArmConstants.highDropPosition.smartMotionMaxAccel()),
                        new MoveGripperCommand(
                                RobotContainer.getGripperSubsystem(),
                                RobotContainer.getArmSubsystem(),
                                MoveGripperCommand.GripperState.OPENED),
                        new MoveArmCommand(
                                RobotContainer.armSubsystem,
                                ArmConstants.highDropPosition.armCmdPos(),
                                ArmConstants.highDropPosition.smartMotionMaxVel(),
                                ArmConstants.highDropPosition.smartMotionMaxAccel())),
                new WaitCommand(0.2),
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
                        new DriveFollowPath("BlueSidePath_2", 1, 2, false),
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
