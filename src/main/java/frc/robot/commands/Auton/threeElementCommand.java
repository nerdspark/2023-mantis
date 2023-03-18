// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import frc.robot.commands.AprTagCommand;
import frc.robot.commands.DriveFollowPath;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.PoseEstimatorSubSystem;
import frc.robot.subsystems.SwerveSubsystem;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** An example command that uses an example subsystem. */
public class threeElementCommand extends SequentialCommandGroup {

  // public line2metersCommand(SwerveSubsystem swervesubsystem) {
  // }

  public threeElementCommand(SwerveSubsystem swerveSubsystem){

    addCommands(
      new DriveFollowPath("threeConePathOne", 3, 2, true),
      new DriveFollowPath("threeConePathTwo", 3, 2, false),
      new DriveFollowPath("threeConePathThree", 3, 2, false),
      new DriveFollowPath("threeConePathFour", 3, 2, false)
    );
  }


}
