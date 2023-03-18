package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ScoreMidPositionCommand extends SequentialCommandGroup {
    public ScoreMidPositionCommand(ArmSubsystem armSubsystem, ElevatorSubsystem elevatorSubsystem,
            WristSubsystem wristSubsystem) {
        addCommands(new MoveArmCommand(armSubsystem, ArmConstants.scoreMidPosition.get("armCmdPos"),
                ArmConstants.scoreMidPosition.get("smartMotionMaxVel"),
                ArmConstants.scoreMidPosition.get("smartMotionMaxAccel")),
                new ParallelCommandGroup(
                        new MoveElevatorCommand(elevatorSubsystem,
                                ArmConstants.scoreMidPosition.get("inclinatorCmdPos")),
                        new MoveWristCommand(wristSubsystem,
                                ArmConstants.scoreMidPosition.get("wristCmdPos"))));
    }
}
