package frc.robot.commands.ArmPositionCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.ArmMoveCommands.MoveArmCommand;
import frc.robot.commands.ArmMoveCommands.MoveBucketCommand;
import frc.robot.commands.ArmMoveCommands.MoveElevatorCommand;
import frc.robot.commands.ArmMoveCommands.MoveGripperCommand;
import frc.robot.commands.ArmMoveCommands.MoveWristCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.BucketSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class BucketPickupCommand extends SequentialCommandGroup {
    public BucketPickupCommand(
            ElevatorSubsystem elevatorSubsystem,
            WristSubsystem wristSubsystem,
            BucketSubsystem bucketSubsystem,
            ArmSubsystem armSubsystem,
            GripperSubsystem gripperSubsystem) {
        addCommands(
                new InstantCommand(() -> armSubsystem.setArmPositionState(ArmSubsystem.ArmPosition.BUCKET_PICKUP)),
                new ParallelCommandGroup(
                        new MoveElevatorCommand(elevatorSubsystem, ArmConstants.bucketPickupPosition.elevatorCmdPos()),
                        new MoveWristCommand(wristSubsystem, ArmConstants.bucketPickupPosition.wristCmdPos()),
                        new MoveBucketCommand(bucketSubsystem, MoveBucketCommand.BucketPosition.EXTENDED),
                        new MoveGripperCommand(gripperSubsystem, armSubsystem, MoveGripperCommand.GripperState.OPENED)),
                new MoveArmCommand(
                        armSubsystem,
                        ArmConstants.bucketPickupPosition.armCmdPos(),
                        ArmConstants.bucketPickupPosition.smartMotionMaxVel(),
                        ArmConstants.bucketPickupPosition.smartMotionMaxAccel()));
    }
}
