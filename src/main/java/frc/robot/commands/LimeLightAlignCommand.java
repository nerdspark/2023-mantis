// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.subsystems.LimeLightSubSystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class LimeLightAlignCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final LimeLightSubSystem limeLightSubSystem;
  private final SwerveSubsystem driveTrainSubsystem;
  boolean targetFound = false;
  int count=0;

  Pose2d lastTarget;

  private static final TrapezoidProfile.Constraints DISTANCE_CONSTRAINTS = new TrapezoidProfile.Constraints(4.0, 2.0);
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =
      new TrapezoidProfile.Constraints(4 * Math.PI, 4 * Math.PI);


  private final ProfiledPIDController aimController = new ProfiledPIDController(3.75, 0.0, 0, OMEGA_CONSTRAINTS);
  private final ProfiledPIDController distanceController = new ProfiledPIDController(1.5, 0, 0, DISTANCE_CONSTRAINTS);


  private final LinearFilter distanceFilter = LinearFilter.movingAverage(3);
  private final LinearFilter rotationFilter = LinearFilter.movingAverage(3);

  private final SlewRateLimiter xSlewRateLimiter = new SlewRateLimiter(8.0);
  private final SlewRateLimiter ySlewRateLimiter = new SlewRateLimiter(8.0);

    /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public LimeLightAlignCommand(LimeLightSubSystem limeLightSubSystem, SwerveSubsystem driveTrain ) {
    this.limeLightSubSystem = limeLightSubSystem;
    this.driveTrainSubsystem = driveTrain;
   
    SmartDashboard.putNumber("LimeLightTest Constructor", 0);

    aimController.enableContinuousInput(-Math.PI, Math.PI);
    aimController.setTolerance(2);
    distanceController.setGoal(1); //1 m from the goal
    distanceController.setTolerance(0.05);


    // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(limeLightSubSystem, driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
    // poseProvider.setCurrentPose(limeLightSubSystem.getBotPose());
    // var robotPose = poseProvider.getCurrentPose();
    // omegaController.reset(robotPose.getRotation().getRadians());
    // xController.reset(robotPose.getX());
    // yController.reset(robotPose.getY());
    lastTarget = null;
    distanceFilter.reset();
    rotationFilter.reset();
    SmartDashboard.putNumber("LimeLightTestCommand INIT", 0);

  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

     if(limeLightSubSystem.hasAprilTagTarget()){

      count++;
      SmartDashboard.putNumber("LimeLightAlignCommand Count", count);


      SmartDashboard.putNumber("LimeLightAlignCommand Tx", limeLightSubSystem.getTargetTx());
      SmartDashboard.putNumber("LimeLightAlignCommand Ty", limeLightSubSystem.getTargetTy());
      SmartDashboard.putNumber("LimeLightAlignCommand Ta", limeLightSubSystem.getTargetTa());
      SmartDashboard.putNumber("LimeLightAlignCommand TagId", limeLightSubSystem.getAprilTagId());

   
      SmartDashboard.putNumber("LimeLightAlignCommand botPose X", limeLightSubSystem.getBotPose().getX());
      SmartDashboard.putNumber("LimeLightAlignCommand botPose Y", limeLightSubSystem.getBotPose().getY());
      SmartDashboard.putNumber("LimeLightAlignCommand botPose Angle", limeLightSubSystem.getBotPose().getRotation().getDegrees());



      SmartDashboard.putNumber("LimeLightAlignCommand target_RobotSpace X", limeLightSubSystem.getTargetPose3d_RobotSpace().getX());
      SmartDashboard.putNumber("LimeLightAlignCommand target_RobotSpace Y", limeLightSubSystem.getTargetPose3d_RobotSpace().getY());
      SmartDashboard.putNumber("LimeLightAlignCommand target_RobotSpace Z", limeLightSubSystem.getTargetPose3d_RobotSpace().getZ());
      SmartDashboard.putNumber("LimeLightAlignCommand target_RobotSpace Angle", limeLightSubSystem.getTargetPose3d_RobotSpace().toPose2d().getRotation().getDegrees());

      
      SmartDashboard.putNumber("LimeLightAlignCommand robot_TargetSpace X", limeLightSubSystem.getRobotPose3d_TargetSpace().getX());
      SmartDashboard.putNumber("LimeLightAlignCommand robot_TargetSpace Y", limeLightSubSystem.getRobotPose3d_TargetSpace().getY());
      SmartDashboard.putNumber("LimeLightAlignCommand robot_TargetSpace Z", limeLightSubSystem.getRobotPose3d_TargetSpace().getZ());
      SmartDashboard.putNumber("LimeLightAlignCommand robot_TargetSpace Angle", limeLightSubSystem.getRobotPose3d_TargetSpace().toPose2d().getRotation().getDegrees());
      
      SmartDashboard.putString("LimeLightAlignCommand limelightRetroResults", "before limelightRetroResults");

      var firstTarget = false;
      // If the target is visible, get the new translation. If the target isn't visible we'll use the last known translation.
      
      
      LimelightTarget_Fiducial[] limelightAprTagResults = limeLightSubSystem.getLimeLightResults().targetingResults.targets_Fiducials;
      
      SmartDashboard.putNumber("LimeLightAlignCommand limelightRetroResults after", limelightAprTagResults.length);

      var fidId=0.0;

      if (limelightAprTagResults.length > 0) {
        firstTarget = lastTarget == null;
        lastTarget = limelightAprTagResults[0].getTargetPose_RobotSpace().toPose2d();

        fidId = limelightAprTagResults[0].fiducialID;
      }
 

      if (lastTarget == null) {
        // We've never seen a target
       

          ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSlewRateLimiter.calculate(0.0), ySlewRateLimiter.calculate(0.0), 1.0);
          SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        
          driveTrainSubsystem.setModuleStates(moduleStates);
     
     
        } else {

          // var camToTargetDistance = limeLightSubSystem.getCameraToTargetDistance(limeLightSubSystem.getTargetTy(), fidId));

          var robotToTargetDistance = limeLightSubSystem.getTargetPose3d_RobotSpace().getZ();

          // Get the robot heading, and the robot-relative heading of the target
          var drivetrainHeading = driveTrainSubsystem.getRotation2d();
          var targetHeading = drivetrainHeading.plus(lastTarget.getRotation());

          SmartDashboard.putNumber("LimeLightAlignCommand targetHeading",lastTarget.getRotation().getDegrees());


          if (firstTarget) {
            // On the first iteration, reset the PID controller
            distanceController.reset(robotToTargetDistance);
          }

           // Get corrections from PID controllers
          aimController.setGoal(targetHeading.getRadians());
          var rotationCorrection = aimController.calculate(drivetrainHeading.getRadians());
          var distanceCorrection = -distanceController.calculate(robotToTargetDistance);

          SmartDashboard.putNumber("LimeLightAlignCommand camToTargetDistance", robotToTargetDistance);
          SmartDashboard.putNumber("LimeLightAlignCommand rotationCorrection", rotationCorrection);
          SmartDashboard.putNumber("LimeLightAlignCommand distanceCorrection", distanceCorrection);

        // Rotate the distance measurement so we drive toward the target in X and Y direction, not just robot forward
        var xySpeeds = new Translation2d(distanceFilter.calculate(distanceCorrection), 0).rotateBy(lastTarget.getRotation());

         double xSpeed = xSlewRateLimiter.calculate(xySpeeds.getX());
         double ySpeed = ySlewRateLimiter.calculate(xySpeeds.getY());
        double omegaSpeed = rotationFilter.calculate(rotationCorrection);


        // ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed,ySpeed,omegaSpeed);
        // SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        //        driveTrainSubsystem.setModuleStates(moduleStates);


        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          xSpeed, ySpeed, omegaSpeed, driveTrainSubsystem.getRotation2d());
      SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

      driveTrainSubsystem.setModuleStates(moduleStates);
      

      }


  }
  
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { return distanceController.atGoal() ;
    // && aimController.atGoal();
  
  }
}
