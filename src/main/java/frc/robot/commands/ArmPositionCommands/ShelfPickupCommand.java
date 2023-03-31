package frc.robot.commands.ArmPositionCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ShelfPickupCommand extends SequentialCommandGroup {
    public ShelfPickupCommand(
            ArmSubsystem armSubsystem, ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem) {}
}
