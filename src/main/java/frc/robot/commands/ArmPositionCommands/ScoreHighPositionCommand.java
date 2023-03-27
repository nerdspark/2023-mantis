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

public class ScoreHighPositionCommand extends SequentialCommandGroup {
    public ScoreHighPositionCommand(ArmSubsystem armSubsystem, ElevatorSubsystem elevatorSubsystem,
            WristSubsystem wristSubsystem) {
        armSubsystem.setArmPositionState(ArmSubsystem.ArmPosition.HighDrop);

        addCommands(new MoveArmCommand(armSubsystem, ArmConstants.scoreHighPosition.get("armCmdPos"),
                ArmConstants.scoreHighPosition.get("smartMotionMaxVel"),
                ArmConstants.scoreHighPosition.get("smartMotionMaxAccel")),
                new ParallelCommandGroup(
                        new MoveElevatorCommand(elevatorSubsystem,
                                ArmConstants.scoreHighPosition.get("inclinatorCmdPos")),
                        new MoveWristCommand(wristSubsystem, ArmConstants.scoreHighPosition.get("wristCmdPos"))));
    }

}
