package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveFollowPath;

public class BlueBobFourCone extends SequentialCommandGroup {
    public BlueBobFourCone() {
        addCommands(
                new DriveFollowPath("BlueBobFourCone_1", 1, 1, true),
                new DriveFollowPath("BlueBobFourCone_2", 1, 1, false),
                new DriveFollowPath("BlueBobFourCone_3", 1, 1, false));
    }
}
