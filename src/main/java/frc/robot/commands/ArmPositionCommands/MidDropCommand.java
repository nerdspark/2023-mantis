package frc.robot.commands.ArmPositionCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.ArmMoveCommands.MoveArmCommand;
import frc.robot.commands.ArmMoveCommands.MoveElevatorCommand;
import frc.robot.commands.ArmMoveCommands.MoveWristCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class MidDropCommand extends SequentialCommandGroup {
    public MidDropCommand(
            ArmSubsystem armSubsystem, ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem) {
        addCommands(
                new InstantCommand(() -> armSubsystem.setArmPositionState(ArmSubsystem.ArmPosition.MID_DROP)),
                new MoveArmCommand(
                        armSubsystem,
                        ArmConstants.midDropPosition.armCmdPos(),
                        ArmConstants.midDropPosition.smartMotionMaxVel(),
                        ArmConstants.midDropPosition.smartMotionMaxAccel()),
                new ParallelCommandGroup(
                        new MoveElevatorCommand(elevatorSubsystem, ArmConstants.midDropPosition.elevatorCmdPos()),
                        new MoveWristCommand(wristSubsystem, ArmConstants.midDropPosition.wristCmdPos())));
    }
}
