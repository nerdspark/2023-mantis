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

public class MidDropCommand extends SequentialCommandGroup {
    public MidDropCommand(ArmSubsystem armSubsystem, ElevatorSubsystem elevatorSubsystem,
                          WristSubsystem wristSubsystem) {
        armSubsystem.setArmPositionState(ArmSubsystem.ArmPosition.MidDrop);
        addCommands(new MoveArmCommand(armSubsystem, ArmConstants.midDropPosition.get("armCmdPos"),
                ArmConstants.midDropPosition.get("smartMotionMaxVel"),
                ArmConstants.midDropPosition.get("smartMotionMaxAccel")),
                new ParallelCommandGroup(
                        new MoveElevatorCommand(elevatorSubsystem,
                                ArmConstants.midDropPosition.get("inclinatorCmdPos")),
                        new MoveWristCommand(wristSubsystem,
                                ArmConstants.midDropPosition.get("wristCmdPos"))));
    }
}
