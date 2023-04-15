package frc.robot.commands.ArmMoveCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class MoveElevatorCommand extends CommandBase {
    private final ElevatorSubsystem m_subsystem;
    private final double targetPosition;

    /**
     * Creates a new MoveElevatorCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public MoveElevatorCommand(ElevatorSubsystem subsystem, double targetPosition) {
        m_subsystem = subsystem;
        this.targetPosition = targetPosition;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_subsystem.setPosition(targetPosition);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        double position = m_subsystem.getPosition();

        //        System.out.println("[MoveElevatorCommand] Position: " + position + " Target: " + targetPosition
        //                + " Difference: " + Math.abs(position - targetPosition));

        // todo: don't math.abs everything
        return (Math.abs(Math.abs(position) - Math.abs(targetPosition)) < 7);
    }
}
