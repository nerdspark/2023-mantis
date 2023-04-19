package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.ArmMoveCommands.MoveArmCommand;
import frc.robot.commands.ArmMoveCommands.MoveGripperCommand;
import frc.robot.commands.ArmPositionCommands.HighDropCommand;
import frc.robot.commands.ArmPositionCommands.MidDropCommand;
import frc.robot.commands.DriveFollowPath;

public class DropPreloadedHigh extends SequentialCommandGroup {
    public DropPreloadedHigh(String driveInPathName) {
        addCommands(
                new MoveGripperCommand(
                        RobotContainer.getGripperSubsystem(),
                        RobotContainer.getArmSubsystem(),
                        MoveGripperCommand.GripperState.CLOSED),
                new ParallelCommandGroup(
                        new DriveFollowPath(driveInPathName, 1, 0.5, true),
                        new InstantCommand(
                                () -> RobotContainer.getWristSubsystem().setPositionOverride(true, -2)),
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
                new InstantCommand(() -> {
                    RobotContainer.getGripperSubsystem().setLeftPosition(-15);
                    RobotContainer.getGripperSubsystem().setRightPosition(2);
                }),
                new WaitCommand(0.5),
                new MidDropCommand(
                        RobotContainer.getArmSubsystem(),
                        RobotContainer.getElevatorSubsystem(),
                        RobotContainer.getWristSubsystem()),
                new InstantCommand(() -> RobotContainer.getWristSubsystem().setPositionOverride(false)));
    }
}
