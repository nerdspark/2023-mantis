package frc.robot.commands.ArmPositionCommands;

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
    public BucketPickupCommand(ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem,
            BucketSubsystem bucketSubsystem, ArmSubsystem armSubsystem, GripperSubsystem gripperSubsystem) {
        armSubsystem.setArmPositionState(ArmSubsystem.ArmPosition.BUCKET_PICKUP);
        addCommands(
            new ParallelCommandGroup(
                new MoveElevatorCommand(elevatorSubsystem, ArmConstants.bucketPickupPosition.get("inclinatorCmdPos")),
                new MoveWristCommand(wristSubsystem, ArmConstants.bucketPickupPosition.get("wristCmdPos")),
                new MoveBucketCommand(bucketSubsystem, MoveBucketCommand.BucketPosition.EXTENDED),
                new MoveGripperCommand(gripperSubsystem, armSubsystem, MoveGripperCommand.GripperState.OPENED)),
            new MoveArmCommand(armSubsystem, ArmConstants.bucketPickupPosition.get("armCmdPos"),
                ArmConstants.bucketPickupPosition.get("smartMotionMaxVel"),
                ArmConstants.bucketPickupPosition.get("smartMotionMaxAccel")));
    }
}
