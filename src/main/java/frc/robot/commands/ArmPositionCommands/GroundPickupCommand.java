package frc.robot.commands.ArmPositionCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
    public GroundPickupCommand(ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem, ArmSubsystem armSubsystem, GripperSubsystem gripperSubsystem) {
        armSubsystem.setArmPositionState(ArmSubsystem.ArmPosition.GROUND_PICKUP);

        addCommands(
            new MoveGripperCommand(gripperSubsystem, armSubsystem, MoveGripperCommand.GripperState.CLOSED),
            new ParallelCommandGroup(
                new MoveArmCommand(armSubsystem, ArmConstants.groundPickupPosition.get("armCmdPos"),
                        ArmConstants.groundPickupPosition.get("smartMotionMaxVel"),
                        ArmConstants.groundPickupPosition.get("smartMotionMaxAccel")),
                new MoveElevatorCommand(elevatorSubsystem, ArmConstants.groundPickupPosition.get("inclinatorCmdPos")),
                new MoveWristCommand(wristSubsystem, ArmConstants.groundPickupPosition.get("wristCmdPos"))),
            new MoveGripperCommand(gripperSubsystem, armSubsystem, MoveGripperCommand.GripperState.OPENED));
    }
}

