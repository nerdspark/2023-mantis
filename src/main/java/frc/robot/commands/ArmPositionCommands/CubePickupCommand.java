package frc.robot.commands.ArmPositionCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.ArmMoveCommands.MoveArmCommand;
import frc.robot.commands.ArmMoveCommands.MoveElevatorCommand;
import frc.robot.commands.ArmMoveCommands.MoveGripperCommand;
import frc.robot.commands.ArmMoveCommands.MoveWristCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class CubePickupCommand extends ParallelCommandGroup {
    public CubePickupCommand(
            ElevatorSubsystem elevatorSubsystem,
            WristSubsystem wristSubsystem,
            ArmSubsystem armSubsystem,
            GripperSubsystem gripperSubsystem) {
        addCommands(
                new InstantCommand(() -> armSubsystem.setArmPositionState(ArmSubsystem.ArmPosition.CUBE_PICKUP)),
                new MoveArmCommand(
                        armSubsystem,
                        ArmConstants.cubePickupPosition.armCmdPos(),
                        ArmConstants.cubePickupPosition.smartMotionMaxVel(),
                        ArmConstants.cubePickupPosition.smartMotionMaxAccel()),
                new MoveElevatorCommand(elevatorSubsystem, ArmConstants.cubePickupPosition.elevatorCmdPos()),
                new MoveWristCommand(wristSubsystem, ArmConstants.cubePickupPosition.wristCmdPos()),
                new MoveGripperCommand(gripperSubsystem, armSubsystem, MoveGripperCommand.GripperState.OPENED));
    }
}
