// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AprTagCommand;
import frc.robot.commands.DriveFollowPath;
import frc.robot.commands.GoToTagCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.PoseEstimatorSubSystem;
import frc.robot.subsystems.SwerveSubsystem;
import org.photonvision.PhotonCamera;

/** An example command that uses an example subsystem. */
public class visionTest5M extends SequentialCommandGroup {

    // public line2metersCommand(SwerveSubsystem swervesubsystem) {
    // }

    public visionTest5M(
            SwerveSubsystem swerveSubsystem,
            PhotonCamera photonCamera,
            ExampleSubsystem mExamplesubsystem,
            PoseEstimatorSubSystem poseEstimator) {

        addCommands(
                new ParallelDeadlineGroup(
                                new AprTagCommand(photonCamera, mExamplesubsystem, 8, poseEstimator::getCurrentPose),
                                new DriveFollowPath("visionTest5M", 1, 0.5, true))
                        .andThen(new GoToTagCommand(photonCamera, swerveSubsystem, poseEstimator::getCurrentPose, 8))

                // new AprTagCommand(photonCamera, mExamplesubsystem, 8, poseEstimator::getCurrentPose),
                // new DriveFollowPath("visionTest5M", 1, 0.5, true)),
                // new AprTagCommand(photonCamera, mExamplesubsystem, 8, poseEstimator::getCurrentPose)
                );
    }
}
