// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DriveToPoseCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveSubsystem drivetrainSubsystem;
  private final Supplier<Pose2d> poseProvider;
  private final Pose2d goalPose;

  private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(VisionConstants.MAX_VELOCITY, VisionConstants.MAX_ACCELARATION);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(VisionConstants.MAX_VELOCITY, VisionConstants.MAX_ACCELARATION);
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRATINTS = 
      new TrapezoidProfile.Constraints(VisionConstants.MAX_VELOCITY_ROTATION, VisionConstants.MAX_ACCELARATION_ROTATION);
  private final ProfiledPIDController xController = new ProfiledPIDController(VisionConstants.kPXController,VisionConstants.kIXController,VisionConstants.kIXController, X_CONSTRAINTS);
  private final ProfiledPIDController yController = new ProfiledPIDController(VisionConstants.kPYController, VisionConstants.kIYController, VisionConstants.kDYController, Y_CONSTRAINTS);
  private final ProfiledPIDController omegaController = new ProfiledPIDController(VisionConstants.kPThetaController,VisionConstants.kIThetaController,VisionConstants.kDThetaController, OMEGA_CONSTRATINTS);

  boolean targetReached = false;
  boolean goalSet;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveToPoseCommand(SwerveSubsystem drivetrainSubsystem, Supplier<Pose2d> poseProvider, Pose2d goalPose) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.poseProvider = poseProvider;
    this.goalPose = goalPose;
    SmartDashboard.putNumber("FindEstimatedPoseCommand Constructor", 0);
   
    xController.setTolerance(VisionConstants.TRANSLATION_TOLERANCE);
    yController.setTolerance(VisionConstants.TRANSLATION_TOLERANCE);
    omegaController.setTolerance(VisionConstants.ROTATION_TOLERANCE);
    omegaController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      SmartDashboard.putNumber("AprTagPipeLine INIT", 0); 

      var robotPose = poseProvider.get();
      omegaController.reset(robotPose.getRotation().getRadians());
      xController.reset(robotPose.getX());
      yController.reset(robotPose.getY());
  
       
      omegaController.setGoal(goalPose.getRotation().getRadians());
      xController.setGoal(goalPose.getX());
      yController.setGoal(goalPose.getY());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    var robotPose = poseProvider.get();
    SmartDashboard.putNumber("DriveToPoseCommand robotPose.X", robotPose.getX());
    SmartDashboard.putNumber("DriveToPoseCommand robotPose.Y", robotPose.getY());
    SmartDashboard.putNumber("DriveToPoseCommand robotPose.Angle", robotPose.getRotation().getRadians());

    SmartDashboard.putNumber("DriveToPoseCommand goalPose.X", goalPose.getX());
    SmartDashboard.putNumber("DriveToPoseCommand goalPose.Y", goalPose.getY());
    SmartDashboard.putNumber("DriveToPoseCommand goalPose.Angle", goalPose.getRotation().getRadians());


      var xSpeed = xController.calculate(robotPose.getX());
      if (xController.atGoal()) {
        xSpeed = 0;
        
      }
      
      var ySpeed = yController.calculate(robotPose.getY());
      if (yController.atGoal()) {
        ySpeed = 0;
      }
  
      var omegaSpeed = omegaController.calculate(robotPose.getRotation().getRadians());
      if (omegaController.atGoal()) {
        omegaSpeed = 0;
      }
  
      ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          xSpeed, ySpeed, omegaSpeed, drivetrainSubsystem.getRotation2d());
      SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
       drivetrainSubsystem.setModuleStates(moduleStates);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
       return xController.atGoal() && yController.atGoal() && omegaController.atGoal();
  }
}
