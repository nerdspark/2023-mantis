package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class GroundPickupCommand extends SequentialCommandGroup {
    public GroundPickupCommand(ArmSubsystem armSubsystem, ElevatorSubsystem elevatorSubsystem,
            WristSubsystem wristSubsystem) {

    }
}
