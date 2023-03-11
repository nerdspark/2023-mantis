// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.BucketSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.WristSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class HomePositionCommand extends CommandBase {
  private final ArmSubsystem armSubsystem;
  private final GripperSubsystem gripperSubsystem;
  private final BucketSubsystem bucketSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private final WristSubsystem wristSubsystem;
  /** Creates a new HomePositionCommand. */
  public HomePositionCommand(ArmSubsystem armSubsystem, GripperSubsystem gripperSubsystem, BucketSubsystem bucketSubsystem,
  ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem) {
    this.armSubsystem = armSubsystem;
    this.gripperSubsystem = gripperSubsystem;
    this.bucketSubsystem = bucketSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.wristSubsystem = wristSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
