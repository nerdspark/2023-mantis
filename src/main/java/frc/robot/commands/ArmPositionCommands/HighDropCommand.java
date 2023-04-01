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

public class HighDropCommand extends SequentialCommandGroup {
    public HighDropCommand(
            ArmSubsystem armSubsystem, ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem) {
        addCommands(
                new InstantCommand(() -> armSubsystem.setArmPositionState(ArmSubsystem.ArmPosition.HIGH_DROP)),
                new MoveArmCommand(
                        armSubsystem,
                        ArmConstants.highDropPosition.armCmdPos(),
                        ArmConstants.highDropPosition.smartMotionMaxVel(),
                        ArmConstants.highDropPosition.smartMotionMaxAccel()),
                new ParallelCommandGroup(
                        new MoveElevatorCommand(elevatorSubsystem, ArmConstants.highDropPosition.elevatorCmdPos()),
                        new MoveWristCommand(wristSubsystem, ArmConstants.highDropPosition.wristCmdPos())));
    }
}
