package frc.robot.commands;

import frc.robot.subsystems.WristSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveWristCommand extends CommandBase {
  private final WristSubsystem m_subsystem;
  private final double targetPosition;

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
    m_subsystem.setPosition(targetPosition);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    double position = m_subsystem.getPosition();

    return Math.abs(position - targetPosition) < 5;
  }
}
