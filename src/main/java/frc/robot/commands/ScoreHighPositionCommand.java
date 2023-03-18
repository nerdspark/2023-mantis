package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ScoreHighPositionCommand extends SequentialCommandGroup {
    public ScoreHighPositionCommand(ArmSubsystem armSubsystem, ElevatorSubsystem elevatorSubsystem,
            WristSubsystem wristSubsystem) {
        addCommands(new MoveArmCommand(armSubsystem, ArmConstants.scoreHighPosition.get("armCmdPos"),
                ArmConstants.scoreHighPosition.get("smartMotionMaxVel"),
                ArmConstants.scoreHighPosition.get("smartMotionMaxAccel")),
                new ParallelCommandGroup(
                        new MoveElevatorCommand(elevatorSubsystem,
                                ArmConstants.scoreHighPosition.get("inclinatorCmdPos")),
                        new MoveWristCommand(wristSubsystem, ArmConstants.scoreHighPosition.get("wristCmdPos"))));
    }

}
