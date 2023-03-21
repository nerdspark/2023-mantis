package frc.robot.commands;

import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OffsetFromTargetAprTag;
import frc.robot.Constants.VisionConstants;


public class GoToTagCommand extends CommandBase {
  
  private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(VisionConstants.MAX_VELOCITY, VisionConstants.MAX_ACCELARATION);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(VisionConstants.MAX_VELOCITY, VisionConstants.MAX_ACCELARATION);
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRATINTS = 
      new TrapezoidProfile.Constraints(VisionConstants.MAX_VELOCITY_ROTATION, VisionConstants.MAX_ACCELARATION_ROTATION);
  
  private  int tagToAlign;
  //private static final Transform2d TAG_TO_GOAL = new Transform2d(new Translation2d(0.5, 0), Rotation2d.fromDegrees(180.0));
  // private static final Transform3d TAG_TO_GOAL = 
  //     new Transform3d(
  //         new Translation3d(1, 0.0, 0.0),
  //         new Rotation3d(0.0, 0.0, Math.PI));

  private static final Transform2d TAG_TO_GOAL = new Transform2d(new Translation2d(1.0, 0), Rotation2d.fromDegrees(180.0));
  private   OffsetFromTargetAprTag offsetFromTarget = OffsetFromTargetAprTag.CENTER;
  private   Transform2d GOAL_OFFSET = null;

  private final PhotonCamera photonCamera;
  private final SwerveSubsystem drivetrainSubsystem;
  private final Supplier<Pose2d> poseProvider;

  private final ProfiledPIDController xController = new ProfiledPIDController(VisionConstants.kPXController,VisionConstants.kIXController,VisionConstants.kIXController, X_CONSTRAINTS);
  private final ProfiledPIDController yController = new ProfiledPIDController(VisionConstants.kPYController, VisionConstants.kIYController, VisionConstants.kDYController, Y_CONSTRAINTS);
  private final ProfiledPIDController omegaController = new ProfiledPIDController(VisionConstants.kPThetaController,VisionConstants.kIThetaController,VisionConstants.kDThetaController, OMEGA_CONSTRATINTS);

  private PhotonTrackedTarget lastTarget;

  boolean targetReached = false;


  public GoToTagCommand(
        PhotonCamera photonCamera, 
        SwerveSubsystem drivetrainSubsystem,
        Supplier<Pose2d> poseProvider, int tagToAlign, OffsetFromTargetAprTag offsetFromTarget) {
    this.photonCamera = photonCamera;
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.poseProvider = poseProvider;
    this.tagToAlign=tagToAlign;
    this.offsetFromTarget = offsetFromTarget;
    

    this.GOAL_OFFSET = new Transform2d(new Translation2d(this.offsetFromTarget.xOffset,this.offsetFromTarget.yOffset), Rotation2d.fromDegrees(this.offsetFromTarget.rotationOffset));


    xController.setTolerance(VisionConstants.TRANSLATION_TOLERANCE);
    yController.setTolerance(VisionConstants.TRANSLATION_TOLERANCE);
    omegaController.setTolerance(VisionConstants.ROTATION_TOLERANCE);
    omegaController.enableContinuousInput(-Math.PI, Math.PI);
   
    addRequirements(drivetrainSubsystem);
  }
  public GoToTagCommand(
    PhotonCamera photonCamera, 
    SwerveSubsystem drivetrainSubsystem,
    Supplier<Pose2d> poseProvider, int tagToAlign){

      this(photonCamera, drivetrainSubsystem, poseProvider, tagToAlign, OffsetFromTargetAprTag.CENTER );

    }
  

  @Override
  public void initialize() {
    lastTarget = null;
    var robotPose = poseProvider.get();

    SmartDashboard.putNumber("TagChaseInit robotPose.X", robotPose.getX());
    SmartDashboard.putNumber("TagChaseInit robotPose.Y", robotPose.getY());
    SmartDashboard.putNumber("TagChaseInit robotPose.Angle", robotPose.getRotation().getRadians());


    omegaController.reset(robotPose.getRotation().getRadians());
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());

    targetReached = false;

  }

  @Override
  public void execute() {

    var robotPose2d = poseProvider.get();

    var robotPose = 
    new Pose3d(
        robotPose2d.getX(),
        robotPose2d.getY(),
        0.0, 
        new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));

    SmartDashboard.putNumber("GoToTagCommand robotPose.X", robotPose.getX());
    SmartDashboard.putNumber("GoToTagCommand robotPose.Y", robotPose.getY());
    SmartDashboard.putNumber("GoToTagCommand robotPose.Angle", robotPose2d.getRotation().getRadians());
    var photonRes = photonCamera.getLatestResult();
    SmartDashboard.putBoolean("Target Found?", photonRes.hasTargets());

    if (photonRes.hasTargets()) {
      // Find the tag we want to chase

      Optional<PhotonTrackedTarget> targetOpt = null;
      // if(targetFound == false) {
        targetOpt = photonRes.getTargets().stream()
          .filter(t -> t.getFiducialId() == this.tagToAlign)
          .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() <= .5 && t.getPoseAmbiguity() != -1)
          .findFirst();
      // }
      
      if(targetOpt != null){
      if (targetOpt.isPresent()) {
        var target = targetOpt.get();
        if (!target.equals(lastTarget)) {
          // This is new target data, so recalculate the goal
          lastTarget = target;

          // Transform the robot's pose to find the camera's pose
          var cameraPose = robotPose.transformBy(Constants.VisionConstants.APRILTAG_CAMERA_TO_ROBOT.inverse());

          // Trasnform the camera's pose to the target's pose
          var camToTarget = target.getBestCameraToTarget();
          var targetPose = cameraPose.transformBy(camToTarget);
          
          // Transform the tag's pose to set our goal
          // var goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();
          var centerGoalPose = targetPose.toPose2d().transformBy(TAG_TO_GOAL);

          //offset the goal pose by offset from target to align to scoring location.

          var goalPose = centerGoalPose.transformBy(GOAL_OFFSET);

          targetReached = isRobotAtGoalPose(goalPose, robotPose2d);

          // Drive
            xController.setGoal(goalPose.getX()); 
            yController.setGoal(goalPose.getY());
            omegaController.setGoal(goalPose.getRotation().getRadians());
            SmartDashboard.putNumber("GoToTagCommand goal Pose X", goalPose.getX());
            SmartDashboard.putNumber("GoToTagCommand goal Pose Y", goalPose.getY());
            SmartDashboard.putNumber("GoToTagCommand goal Pose Omega", goalPose.getRotation().getRadians());
          
        }      
      }
    }
    }
   
    if (lastTarget == null) {
      // No target has been visible
      drivetrainSubsystem.stopModules();
    } else{   
      
      var xSpeed = xController.calculate(robotPose.getX());
      if (xController.atGoal()) {
        xSpeed = 0;
      }

      var ySpeed = yController.calculate(robotPose.getY());

      if (yController.atGoal()) {
        ySpeed = 0;
      }

      var omegaSpeed = omegaController.calculate(robotPose2d.getRotation().getRadians());

      if (omegaController.atGoal()) {
        omegaSpeed = 0;
      }

      SmartDashboard.putNumber("GoToTagCommand  X Speed", xSpeed);
      SmartDashboard.putNumber("GoToTagCommand  Y Speed", ySpeed);
      SmartDashboard.putNumber("GoToTagCommand Omega Speed", omegaSpeed);

      ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          xSpeed, ySpeed, omegaSpeed, drivetrainSubsystem.getRotation2d());
      SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

      drivetrainSubsystem.setModuleStates(moduleStates);
    }
  
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stopModules();
  }


  // // Returns true when the command should end.
  @Override
  public boolean isFinished() {
 
    // return xController.atGoal() && yController.atGoal() && omegaController.atGoal();
    return targetReached || lastTarget==null;
    
    }

  private boolean isRobotAtGoalPose(Pose2d goalPose, Pose2d robotPose){


    Transform2d poseDifference = goalPose.minus(robotPose);

    SmartDashboard.putNumber("GoToTagCommand poseDifferenceX", poseDifference.getX());
    SmartDashboard.putNumber("GoToTagCommand poseDifference Y", poseDifference.getY());
    SmartDashboard.putNumber("GoToTagCommand poseDifference Angle", poseDifference.getRotation().getDegrees());

    return (Math.abs(poseDifference.getX()) <= VisionConstants.TRANSLATION_TOLERANCE) && 
        (Math.abs(poseDifference.getY()) <= VisionConstants.TRANSLATION_TOLERANCE) && 
        (Math.abs(poseDifference.getRotation().getDegrees()) <= VisionConstants.ROTATION_TOLERANCE);
  }

  }
