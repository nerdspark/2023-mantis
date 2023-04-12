package frc.robot.commands.ArmPositionCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.ArmMoveCommands.MoveArmCommand;
import frc.robot.commands.ArmMoveCommands.MoveElevatorCommand;
import frc.robot.commands.ArmMoveCommands.MoveGripperCommand;
import frc.robot.commands.ArmMoveCommands.MoveWristCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class GroundPickupCommand extends SequentialCommandGroup {
    public GroundPickupCommand(
            ElevatorSubsystem elevatorSubsystem,
            WristSubsystem wristSubsystem,
            ArmSubsystem armSubsystem,
            GripperSubsystem gripperSubsystem) {
        addCommands(
                new InstantCommand(() -> armSubsystem.setArmPositionState(ArmSubsystem.ArmPosition.GROUND_PICKUP)),
                new MoveGripperCommand(gripperSubsystem, armSubsystem, MoveGripperCommand.GripperState.CLOSED),
                new ParallelCommandGroup(
                        new MoveArmCommand(
                                armSubsystem,
                                ArmConstants.groundPickupPosition.armCmdPos(),
                                ArmConstants.groundPickupPosition.smartMotionMaxVel(),
                                ArmConstants.groundPickupPosition.smartMotionMaxAccel()),
                        new MoveElevatorCommand(elevatorSubsystem, ArmConstants.groundPickupPosition.elevatorCmdPos()),
                        new WaitCommand(0.3)
                                .andThen(new MoveWristCommand(
                                        wristSubsystem, ArmConstants.groundPickupPosition.wristCmdPos()))),
                new MoveGripperCommand(gripperSubsystem, armSubsystem, MoveGripperCommand.GripperState.OPENED));
    }
}
