package frc.robot.commands.ArmMoveCommands;

import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class MoveGripperCommand extends CommandBase {
  private final GripperSubsystem m_subsystem;
  private final ArmSubsystem armSubsystem;

  public enum GripperState {
      OPENED,
      CLOSED,
  }

  private double leftPosition;
  private double rightPosition;
  private GripperState state;


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
  public void initialize() {
  }

  @Override
  public void execute() {
    String leftKey = "";
    String rightKey = "";

    if (state == GripperState.OPENED) {
      leftKey = "leftGripperOpenCmdPos";
      rightKey = "rightGripperOpenCmdPos";
    } else {
      leftKey = "leftGripperCloseCmdPos";
      rightKey = "rightGripperCloseCmdPos";
    }
    
    switch (armSubsystem.getArmPositionState()) {
      case HOME:
          leftPosition = ArmConstants.homePosition.get(leftKey);
          rightPosition = ArmConstants.homePosition.get(rightKey);
          break;
      case BUCKET_PICKUP:
          leftPosition = ArmConstants.bucketPickupPosition.get(leftKey);
          rightPosition = ArmConstants.bucketPickupPosition.get(rightKey);
          break;
      case GROUND_PICKUP:
          leftPosition = ArmConstants.groundPickupPosition.get(leftKey);
          rightPosition = ArmConstants.groundPickupPosition.get(rightKey);
          break;
      case SHELF_PICKUP:
          leftPosition = ArmConstants.intakeShelfPosition.get(leftKey);
          rightPosition = ArmConstants.intakeShelfPosition.get(rightKey);
          break;
      case HIGH_DROP:
          leftPosition = ArmConstants.highDropPosition.get(leftKey);
          rightPosition = ArmConstants.highDropPosition.get(rightKey);
          break;
      case MID_DROP:
          leftPosition = ArmConstants.midDropPosition.get(leftKey);
          rightPosition = ArmConstants.midDropPosition.get(rightKey);
          break;
      case GROUND_DROP:
          leftPosition = ArmConstants.groundDropPosition.get(leftKey);
          rightPosition = ArmConstants.groundDropPosition.get(rightKey);
          break;
      default:
        break;
    }

    m_subsystem.setLeftPosition(leftPosition);
    m_subsystem.setRightPosition(rightPosition);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    System.out.println("[MoveArmCommand] Left: " + m_subsystem.getLeftPosition() + " Right: " + m_subsystem.getRightPosition() + " Target: " + leftPosition
        + " Difference: " + Math.abs(m_subsystem.getLeftPosition() - leftPosition));                        
    // return (Math.abs(Math.abs(m_subsystem.getLeftPosition()) - Math.abs(leftPosition)) < 2
    //     && Math.abs(Math.abs(m_subsystem.getRightPosition()) - Math.abs(rightPosition)) < 2);
    return true;
  }
}
