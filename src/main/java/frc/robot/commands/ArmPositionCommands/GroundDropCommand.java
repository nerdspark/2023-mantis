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

public class GroundDropCommand extends SequentialCommandGroup {
    public GroundDropCommand(
            ArmSubsystem armSubsystem, ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem) {
        addCommands(
                new InstantCommand(() -> armSubsystem.setArmPositionState(ArmSubsystem.ArmPosition.GROUND_DROP)),
                new MoveArmCommand(
                        armSubsystem,
                        ArmConstants.groundDropPosition.armCmdPos(),
                        ArmConstants.groundDropPosition.smartMotionMaxVel(),
                        ArmConstants.groundDropPosition.smartMotionMaxAccel()),
                new ParallelCommandGroup(
                        new MoveElevatorCommand(elevatorSubsystem, ArmConstants.groundDropPosition.elevatorCmdPos()),
                        new MoveWristCommand(wristSubsystem, ArmConstants.groundDropPosition.wristCmdPos())));
    }
}
