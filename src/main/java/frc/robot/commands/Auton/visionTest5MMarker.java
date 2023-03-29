// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.AprTagCommand;
import frc.robot.commands.DriveFollowPath;
import frc.robot.commands.GoToTagCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.PoseEstimatorSubSystemOld;
import frc.robot.subsystems.SwerveSubsystem;
import org.photonvision.PhotonCamera;

/** An example command that uses an example subsystem. */
public class visionTest5MMarker extends SequentialCommandGroup {

    // public line2metersCommand(SwerveSubsystem swervesubsystem) {
    // }

    public visionTest5MMarker(
            SwerveSubsystem swerveSubsystem,
            PhotonCamera photonCamera,
            ExampleSubsystem mExamplesubsystem,
            PoseEstimatorSubSystem poseEstimator) {

        AutoConstants.autoEventMap.put("marker1", new PrintCommand("Passed Marker 1"));

        addCommands(
                new ParallelDeadlineGroup(
                                new AprTagCommand(photonCamera, mExamplesubsystem, 8, poseEstimator::getCurrentPose),
                                new DriveFollowPath("visionTest5M", 1, 0.5, true))
                        .andThen(new GoToTagCommand(photonCamera, swerveSubsystem, poseEstimator::getCurrentPose, 8))

                // new FollowPathWithEvents(new DriveFollowPath("visionTest5M", 1, 0.5, false),
                //   DriveFollowPath.get(0).getMarkers(), autoEventMap)

                // new AprTagCommand(photonCamera, mExamplesubsystem, 8, poseEstimator::getCurrentPose),
                // new DriveFollowPath("visionTest5M", 1, 0.5, true)),
                // new AprTagCommand(photonCamera, mExamplesubsystem, 8, poseEstimator::getCurrentPose)
                );
    }
}
