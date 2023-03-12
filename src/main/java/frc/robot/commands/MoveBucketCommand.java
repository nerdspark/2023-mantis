package frc.robot.commands;

import frc.robot.subsystems.BucketSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveBucketCommand extends CommandBase {
  private final BucketSubsystem m_subsystem;
  private final BucketPosition position;

  public static enum BucketPosition {
    EXTENDED,
    RETRACTED
  }

  /**
   * Creates a new MoveBucketCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveBucketCommand(BucketSubsystem subsystem, BucketPosition position) {
    m_subsystem = subsystem;
    this.position = position;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (position == BucketPosition.EXTENDED) {
      m_subsystem.extend();
    } else if (position == BucketPosition.RETRACTED) {
      m_subsystem.retract();
    }
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
