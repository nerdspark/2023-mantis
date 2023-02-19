// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import frc.robot.commands.DriveFollowPath;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** An example command that uses an example subsystem. */
public class line2metersCommand extends SequentialCommandGroup {

  public line2metersCommand(){

    addCommands(
      new DriveFollowPath("line2meters", 1, 0.5)
      
    );
  }
}
