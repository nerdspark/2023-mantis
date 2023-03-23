// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmPositionCommands;

import frc.robot.Constants.ArmConstants;

import java.util.function.Supplier;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmPosition;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class MicroAdjustCommand extends CommandBase {
  private final WristSubsystem wristSubsystem;
  private final ArmSubsystem armSubsystem;
  private final Supplier<Double> leftJoystickY;
  private final Supplier<Double> rightJoystickY;

  /** Creates a new MicroAdjustCommand. */
  public MicroAdjustCommand(ArmSubsystem armSubsystem, WristSubsystem wristSubsystem, Supplier<Double> leftJoystickY, Supplier<Double> rightJoystickY) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.wristSubsystem = wristSubsystem;
    this.armSubsystem = armSubsystem;
    this.leftJoystickY = leftJoystickY;
    this.rightJoystickY = rightJoystickY;
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentArmPositionState = 0;
    double currentWristPositionState = 0;

    ArmPosition armPositionState = armSubsystem.getArmPositionState();

    switch (armPositionState) {
      case GroundDrop:
        currentArmPositionState = ArmConstants.intakeGroundPosition.get("armCmdPos");
        currentWristPositionState = ArmConstants.intakeGroundPosition.get("wristCmdPos");
        break;
      case HighDrop:
        currentArmPositionState = ArmConstants.scoreHighPosition.get("armCmdPos");
        currentWristPositionState = ArmConstants.scoreHighPosition.get("wristCmdPos");
        break;
      case MidDrop:
        currentArmPositionState = ArmConstants.scoreMidPosition.get("armCmdPos");
        currentWristPositionState = ArmConstants.scoreMidPosition.get("wristCmdPos");
        break;
      default:
        return;
    }

    if (Math.abs(rightJoystickY.get()) > 0.05) {
      armSubsystem.goToPosition(currentArmPositionState + rightJoystickY.get() * ArmConstants.armAdjustMultiplier);
    } else {
      armSubsystem.goToPosition(currentArmPositionState);
    }

    if (Math.abs(leftJoystickY.get()) > 0.05) {
      wristSubsystem.setPosition(currentWristPositionState + leftJoystickY.get() * ArmConstants.wristAdjustMultiplier);
    } else {
      wristSubsystem.setPosition(currentWristPositionState);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
