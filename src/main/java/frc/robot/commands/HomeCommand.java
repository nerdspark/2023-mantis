package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class HomeCommand extends SequentialCommandGroup {
    public HomeCommand(ArmSubsystem armSubsystem, ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem, GripperSubsystem gripperSubsystem) {
        addCommands(new ParallelCommandGroup(
                new MoveWristCommand(wristSubsystem, ArmConstants.homePosition.get("wristCmdPos")),
                new MoveElevatorCommand(elevatorSubsystem, ArmConstants.homePosition.get("inclinatorCmdPos")),
                new MoveGripperCommand(gripperSubsystem, armSubsystem, MoveGripperCommand.GripperState.Closed)),
                new MoveArmCommand(armSubsystem, ArmConstants.homePosition.get("armCmdPos"),
                        ArmConstants.homePosition.get("smartMotionMaxVel"),
                        ArmConstants.homePosition.get("smartMotionMaxAccel")));
    }
}
