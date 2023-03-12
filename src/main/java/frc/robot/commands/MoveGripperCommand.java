package frc.robot.commands;

import frc.robot.subsystems.GripperSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class MoveGripperCommand extends CommandBase {
  private final GripperSubsystem m_subsystem;
  private final double leftPosition;
  private final double rightPosition;


  /**
   * Creates a new MoveGripperCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveGripperCommand(GripperSubsystem subsystem, double leftPosition, double rightPosition) {
    m_subsystem = subsystem;
    this.leftPosition = leftPosition;
    this.rightPosition = rightPosition;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_subsystem.setLeftPosition(leftPosition);
    m_subsystem.setRightPosition(rightPosition);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    // todo
    return false;
  }
}
