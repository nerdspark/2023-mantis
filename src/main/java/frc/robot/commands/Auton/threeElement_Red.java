// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.AprTagCommand;
import frc.robot.commands.DriveFollowPath;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.commands.ArmMoveCommands.MoveGripperCommand;
import frc.robot.commands.ArmMoveCommands.MoveGripperCommand.GripperState;
import frc.robot.commands.ArmPositionCommands.GroundDropCommand;
import frc.robot.commands.ArmPositionCommands.GroundPickupCommand;
import frc.robot.commands.ArmPositionCommands.HomeCommand;
import frc.robot.commands.ArmPositionCommands.ScoreHighPositionCommand;
import frc.robot.commands.ArmPositionCommands.ScoreMidPositionCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.PoseEstimatorSubSystem;
import frc.robot.subsystems.SwerveSubsystem;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/** An example command that uses an example subsystem. */
public class threeElement_Red extends SequentialCommandGroup {

  // public line2metersCommand(SwerveSubsystem swervesubsystem) {
  // }

  public threeElement_Red(SwerveSubsystem swerveSubsystem){

    addCommands(
      new ScoreHighPositionCommand(RobotContainer.getArmSubsystem(), RobotContainer.getElevatorSubsystem(), RobotContainer.getWristSubsystem()),
      new MoveGripperCommand(RobotContainer.getGripperSubsystem(), RobotContainer.getArmSubsystem(), GripperState.Open),
      new WaitCommand(0.2),
      new ParallelCommandGroup(  
        new DriveFollowPath("threeElementRed_1", 1, 0.5, true),
        new SequentialCommandGroup(
          new WaitCommand(1),
          new ScoreMidPositionCommand(RobotContainer.getArmSubsystem(), RobotContainer.getElevatorSubsystem(), RobotContainer.getWristSubsystem())
        )
      ),
      new GroundPickupCommand(RobotContainer.getElevatorSubsystem(), RobotContainer.getWristSubsystem(), RobotContainer.getArmSubsystem(), RobotContainer.getGripperSubsystem()),
      new MoveGripperCommand(RobotContainer.getGripperSubsystem(), RobotContainer.getArmSubsystem(), GripperState.Closed),

      new WaitCommand(0.2),
      new ParallelCommandGroup(
        new DriveFollowPath("threeElementRed_2", 1, 0.5, false),
        new ScoreMidPositionCommand(RobotContainer.getArmSubsystem(), RobotContainer.getElevatorSubsystem(), RobotContainer.getWristSubsystem())
      ),

      new ScoreHighPositionCommand(RobotContainer.getArmSubsystem(), RobotContainer.getElevatorSubsystem(), RobotContainer.getWristSubsystem()),
      new MoveGripperCommand(RobotContainer.getGripperSubsystem(), RobotContainer.getArmSubsystem(), GripperState.Open),
      new WaitCommand(0.2),
      new ParallelCommandGroup(  
        new DriveFollowPath("threeElementRed_3", 1, 0.5, true),
        new SequentialCommandGroup(
          new WaitCommand(1),
          new ScoreMidPositionCommand(RobotContainer.getArmSubsystem(), RobotContainer.getElevatorSubsystem(), RobotContainer.getWristSubsystem())
        )
      ),
      new GroundPickupCommand(RobotContainer.getElevatorSubsystem(), RobotContainer.getWristSubsystem(), RobotContainer.getArmSubsystem(), RobotContainer.getGripperSubsystem()),
      new MoveGripperCommand(RobotContainer.getGripperSubsystem(), RobotContainer.getArmSubsystem(), GripperState.Closed),

      new WaitCommand(0.2),
      new ParallelCommandGroup(
        new DriveFollowPath("threeElementRed_4", 1, 0.5, false),
        new ScoreMidPositionCommand(RobotContainer.getArmSubsystem(), RobotContainer.getElevatorSubsystem(), RobotContainer.getWristSubsystem())
      )
    );
  }


}
