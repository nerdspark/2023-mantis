package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class GroundDropCommand extends SequentialCommandGroup {
    public GroundDropCommand(ArmSubsystem armSubsystem, ElevatorSubsystem elevatorSubsystem,
            WristSubsystem wristSubsystem) {
        addCommands(
                new MoveArmCommand(armSubsystem, ArmConstants.scoreGroundPosition.get("armCmdPos"),
                        ArmConstants.scoreGroundPosition.get("smartMotionMaxVel"),
                        ArmConstants.scoreGroundPosition.get("smartMotionMaxAccel")),
                new ParallelCommandGroup(
                        new MoveElevatorCommand(elevatorSubsystem,
                                ArmConstants.scoreGroundPosition.get("inclinatorCmdPos")),
                        new MoveWristCommand(wristSubsystem, ArmConstants.scoreGroundPosition.get("wristCmdPos"))));
    }
}
