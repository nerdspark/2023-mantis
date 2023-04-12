package frc.robot.commands.ArmMoveCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArmCommand extends CommandBase {
    private final ArmSubsystem m_subsystem;
    private final double targetPosition;
    private final double maxVelocity;
    private final double maxAccel;

    /**
     * Creates a new MoveArmCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public MoveArmCommand(ArmSubsystem subsystem, double targetPosition, double maxVelocity, double maxAccel) {
        m_subsystem = subsystem;
        this.targetPosition = targetPosition;
        this.maxVelocity = maxVelocity;
        this.maxAccel = maxAccel;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_subsystem.changeArmSmartMotionParameters(maxVelocity, maxAccel);
        m_subsystem.setPosition(targetPosition);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        double[] positions = m_subsystem.getPositions();

        // todo: don't math.abs everything
        //        System.out.println("[MoveArmCommand] Left: "
        //                + positions[0]
        //                + " Right: "
        //                + positions[1]
        //                + " Target: "
        //                + targetPosition
        //                + " Difference: "
        //                + Math.abs(positions[0] - targetPosition));

        return (Math.abs(Math.abs(positions[0]) - Math.abs(targetPosition)) < 7
                && Math.abs(Math.abs(positions[1]) - Math.abs(targetPosition)) < 7);
    }
}
