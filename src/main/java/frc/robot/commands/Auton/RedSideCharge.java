package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.ArmMoveCommands.MoveGripperCommand;
import frc.robot.commands.ArmPositionCommands.BucketPickupCommand;
import frc.robot.commands.ArmPositionCommands.HighDropCommand;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.DriveFollowPath;

public class RedSideCharge extends SequentialCommandGroup {
    public RedSideCharge() {
        addCommands(
                new MoveGripperCommand(
                        RobotContainer.getGripperSubsystem(),
                        RobotContainer.getArmSubsystem(),
                        MoveGripperCommand.GripperState.CLOSED),
                new ParallelCommandGroup(
                        new DriveFollowPath("RedSideCharge_0", 1, 0.5, true),
                        new HighDropCommand(
                                RobotContainer.getArmSubsystem(),
                                RobotContainer.getElevatorSubsystem(),
                                RobotContainer.getWristSubsystem())),
                new WaitCommand(0.1),
                new MoveGripperCommand(
                        RobotContainer.getGripperSubsystem(),
                        RobotContainer.getArmSubsystem(),
                        MoveGripperCommand.GripperState.OPENED),
                new WaitCommand(0.2),
                new ParallelCommandGroup(
                        new BucketPickupCommand(
                                RobotContainer.getElevatorSubsystem(),
                                RobotContainer.getWristSubsystem(),
                                RobotContainer.getBucketSubsystem(),
                                RobotContainer.getArmSubsystem(),
                                RobotContainer.getGripperSubsystem()),
                        new DriveFollowPath("RedSideCharge_1", 1, 1, false)),
                new BalanceCommand(RobotContainer.getSwerveSubsystem(), false));
    }
}
