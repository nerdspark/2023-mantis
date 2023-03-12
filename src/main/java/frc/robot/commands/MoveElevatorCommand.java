package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

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
  public void initialize() {
  }

  @Override
  public void execute() {
    m_subsystem.setPosition(targetPosition);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    double[] positions = m_subsystem.getPositions();

    System.out.println("[MoveElevatorCommand] Left: " + positions[0] + " Right: " + positions[1] + " Target: " + targetPosition
        + " Difference: " + Math.abs(positions[0] - targetPosition));
        
    // todo: don't math.abs everything
    return (Math.abs(Math.abs(positions[0]) - Math.abs(targetPosition)) < 1
        && Math.abs(Math.abs(positions[1]) - Math.abs(targetPosition)) < 1);
  }
}
