package frc.robot.commands.ArmPositionCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.ArmMoveCommands.*;
import frc.robot.subsystems.*;

public class HomeCommand extends SequentialCommandGroup {
    public HomeCommand(
            ArmSubsystem armSubsystem,
            ElevatorSubsystem elevatorSubsystem,
            WristSubsystem wristSubsystem,
            GripperSubsystem gripperSubsystem,
            BucketSubsystem bucketSubsystem) {
        addCommands(
                new InstantCommand(() -> armSubsystem.setArmPositionState(ArmSubsystem.ArmPosition.HOME)),
                new ParallelCommandGroup(
                        new MoveBucketCommand(bucketSubsystem, MoveBucketCommand.BucketPosition.RETRACTED),
                        new MoveWristCommand(wristSubsystem, ArmConstants.homePosition.wristCmdPos()),
                        new MoveElevatorCommand(elevatorSubsystem, ArmConstants.homePosition.elevatorCmdPos()),
                        new MoveGripperCommand(gripperSubsystem, armSubsystem, MoveGripperCommand.GripperState.CLOSED)),
                new MoveArmCommand(
                        armSubsystem,
                        ArmConstants.homePosition.armCmdPos(),
                        ArmConstants.homePosition.smartMotionMaxVel(),
                        ArmConstants.homePosition.smartMotionMaxAccel()));
    }
}
