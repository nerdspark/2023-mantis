// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.subsystems.LimeLightSubSystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class LimeLightTestCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final LimeLightSubSystem limeLightSubSystem;
  int count = 0;
 
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public LimeLightTestCommand(LimeLightSubSystem limeLightSubSystem) {
    this.limeLightSubSystem = limeLightSubSystem;
    SmartDashboard.putNumber("LimeLightTest Constructor", 0);
    // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(limeLightSubSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
    SmartDashboard.putNumber("LimeLightTestCommand INIT", 0);  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    if(limeLightSubSystem.hasAprilTagTarget()){
      SmartDashboard.putNumber( "LimeLightTestCommand",count++);

      SmartDashboard.putNumber("LimeLightCommand Tx", limeLightSubSystem.getTargetTx());
      SmartDashboard.putNumber("LimeLightCommand Ty", limeLightSubSystem.getTargetTy());
      SmartDashboard.putNumber("LimeLightCommand Ta", limeLightSubSystem.getTargetTa());
      SmartDashboard.putNumber("LimeLightCommand TagId", limeLightSubSystem.getAprilTagId());
      
      // double [] robotPose = { limeLightSubSystem.getBotPose().getX(), 
      //   limeLightSubSystem.getBotPose().getY(), limeLightSubSystem.getBotPose().getRotation().getDegrees()};

      SmartDashboard.putNumber("LimeLightCommand botPose X", limeLightSubSystem.getBotPose().getX());
      SmartDashboard.putNumber("LimeLightCommand botPose Y", limeLightSubSystem.getBotPose().getY());
      SmartDashboard.putNumber("LimeLightCommand botPose Angle", limeLightSubSystem.getBotPose().getRotation().getDegrees());


      LimelightTarget_Fiducial aprTag = limeLightSubSystem.getLimeLightResults().targetingResults.targets_Fiducials[0];
      // Pose2d targPoseInRobotSpace = aprTag.getTargetPose_RobotSpace2D();

      // double [] tagPose = {targPoseInRobotSpace.getX(), targPoseInRobotSpace.getY(),targPoseInRobotSpace.getRotation().getDegrees()};
     // SmartDashboard.putNumberArray("LimeLightCommand tagPose", tagPose);
     SmartDashboard.putNumber("LimeLightCommand tagPose X", aprTag.getTargetPose_RobotSpace2D().getX());
     SmartDashboard.putNumber("LimeLightCommand tagPose Y", aprTag.getTargetPose_RobotSpace2D().getY());
     SmartDashboard.putNumber("LimeLightCommand tagPose Angle", aprTag.getTargetPose_RobotSpace2D().getRotation().getDegrees());



    }else{
      SmartDashboard.putNumber( "LimeLightTestCommand No Apr Tag",count++);

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;

  }
}
