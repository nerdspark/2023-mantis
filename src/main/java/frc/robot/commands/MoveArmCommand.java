package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

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
    m_subsystem.goToPosition(targetPosition);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    double[] positions = m_subsystem.getPositions();

    return (Math.abs(positions[0] - targetPosition) < 5 && Math.abs(positions[1] - targetPosition) < 5);
  }
}
