// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveFollowPath;
import frc.robot.subsystems.SwerveSubsystem;

/** An example command that uses an example subsystem. */
public class threeElement_Blue extends SequentialCommandGroup {

    // public line2metersCommand(SwerveSubsystem swervesubsystem) {
    // }

    public threeElement_Blue(SwerveSubsystem swerveSubsystem) {

        addCommands(
                new DriveFollowPath("threeElementBlue_1", 3, 2, true),
                new DriveFollowPath("threeElementBlue_2", 3, 2, false),
                new DriveFollowPath("threeElementBlue_3", 3, 2, false),
                new DriveFollowPath("threeElementBlue_4", 3, 2, false)
                // new DriveFollowPath("threeConePathTwo", 3, 2, false),
                // new DriveFollowPath("threeConePathThree", 3, 2, false),
                // new DriveFollowPath("threeConePathFour", 3, 2, false)
                );
    }
}
