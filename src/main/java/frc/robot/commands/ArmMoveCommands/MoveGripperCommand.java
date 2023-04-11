package frc.robot.commands.ArmMoveCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GripperSubsystem;

public class MoveGripperCommand extends CommandBase {
    private final GripperSubsystem m_subsystem;
    private final ArmSubsystem armSubsystem;
    private final GripperState state;
    private double leftPosition;
    private double rightPosition;

    /**
     * Creates a new MoveGripperCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public MoveGripperCommand(GripperSubsystem subsystem, ArmSubsystem armSubsystem, GripperState state) {
        m_subsystem = subsystem;
        this.armSubsystem = armSubsystem;
        this.state = state;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        switch (state) {
            case OPENED -> {
                leftPosition = armSubsystem.getCurrentArmPositionData().leftGripperOpenCmdPos();
                rightPosition = armSubsystem.getCurrentArmPositionData().rightGripperOpenCmdPos();
            }
            case CLOSED -> {
                leftPosition = armSubsystem.getCurrentArmPositionData().leftGripperCloseCmdPos();
                rightPosition = armSubsystem.getCurrentArmPositionData().rightGripperCloseCmdPos();
            }
        }

        m_subsystem.setLeftPosition(leftPosition);
        m_subsystem.setRightPosition(rightPosition);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        //        System.out.println("[MoveGripperCommand] Left: " + m_subsystem.getLeftPosition() + " Right: "
        //                + m_subsystem.getRightPosition() + " Target: " + leftPosition + " Difference: "
        //                + Math.abs(m_subsystem.getLeftPosition() - leftPosition));

        // return (Math.abs(Math.abs(m_subsystem.getLeftPosition()) - Math.abs(leftPosition)) < 2
        //     && Math.abs(Math.abs(m_subsystem.getRightPosition()) - Math.abs(rightPosition)) < 2);
        return true;
    }

    public enum GripperState {
        OPENED,
        CLOSED,
    }
}
