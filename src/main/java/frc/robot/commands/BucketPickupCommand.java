package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.BucketSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class BucketPickupCommand extends SequentialCommandGroup {
    public BucketPickupCommand(ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem,
            BucketSubsystem bucketSubsystem, ArmSubsystem armSubsystem, GripperSubsystem gripperSubsystem) {
        addCommands(
            new ParallelCommandGroup(
                new MoveElevatorCommand(elevatorSubsystem, ArmConstants.intakeBucketPosition.get("inclinatorCmdPos")),
                new MoveWristCommand(wristSubsystem, ArmConstants.intakeBucketPosition.get("wristCmdPos")),
                new MoveBucketCommand(bucketSubsystem, MoveBucketCommand.BucketPosition.EXTENDED),
                new MoveGripperCommand(gripperSubsystem, armSubsystem, MoveGripperCommand.GripperState.Open)),
            new MoveArmCommand(armSubsystem, ArmConstants.intakeBucketPosition.get("armCmdPos"),
                ArmConstants.intakeBucketPosition.get("smartMotionMaxVel"),
                ArmConstants.intakeBucketPosition.get("smartMotionMaxAccel")));
    }
}
