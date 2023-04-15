package frc.robot.commands.ArmMoveCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;

public class MoveWristCommand extends CommandBase {
    private final WristSubsystem m_subsystem;
    private double targetPosition;

    /**
     * Creates a new MoveWristCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public MoveWristCommand(WristSubsystem subsystem, double targetPosition) {
        m_subsystem = subsystem;
        this.targetPosition = targetPosition;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (m_subsystem.isPositionOverridden()) {
            this.targetPosition = m_subsystem.getOverridenPosition();
        }

        m_subsystem.setPosition(targetPosition);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        double position = m_subsystem.getPosition();

        System.out.println("[MoveWristCommand] Position: " + position + " Target: " + targetPosition + " Difference: "
                + Math.abs(position - targetPosition));

        // todo: don't math.abs everything
        return true;
    }
}
