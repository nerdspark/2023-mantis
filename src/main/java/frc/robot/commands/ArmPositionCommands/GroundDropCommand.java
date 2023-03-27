package frc.robot.commands.ArmPositionCommands;

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
    public GroundDropCommand(ArmSubsystem armSubsystem, ElevatorSubsystem elevatorSubsystem,
            WristSubsystem wristSubsystem) {
        armSubsystem.setArmPositionState(ArmSubsystem.ArmPosition.GroundDrop);
        addCommands(
                new MoveArmCommand(armSubsystem, ArmConstants.groundDropPosition.get("armCmdPos"),
                        ArmConstants.groundDropPosition.get("smartMotionMaxVel"),
                        ArmConstants.groundDropPosition.get("smartMotionMaxAccel")),
                new ParallelCommandGroup(
                        new MoveElevatorCommand(elevatorSubsystem,
                                ArmConstants.groundDropPosition.get("inclinatorCmdPos")),
                        new MoveWristCommand(wristSubsystem, ArmConstants.groundDropPosition.get("wristCmdPos"))));
    }
}
