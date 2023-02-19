package frc.robot.commands;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;


public class ChaseTagCommand extends CommandBase {
  
  private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(1, 0.5);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(1, 0.5);
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRATINTS = 
      new TrapezoidProfile.Constraints(4, 4);
  
  private  int tagToChase;
  //private static final Transform2d TAG_TO_GOAL = new Transform2d(new Translation2d(0.5, 0), Rotation2d.fromDegrees(180.0));
  private static final Transform3d TAG_TO_GOAL = 
      new Transform3d(
          new Translation3d(1, 0.0, 0.0),
          new Rotation3d(0.0, 0.0, Math.PI));

  private final PhotonCamera photonCamera;
  private final SwerveSubsystem drivetrainSubsystem;
  private final Supplier<Pose2d> poseProvider;

  private final ProfiledPIDController xController = new ProfiledPIDController(2.5, 0, 0, X_CONSTRAINTS);
  private final ProfiledPIDController yController = new ProfiledPIDController(2.5, 0, 0, Y_CONSTRAINTS);
  private final ProfiledPIDController omegaController = new ProfiledPIDController(1, 0, 0, OMEGA_CONSTRATINTS);

  private PhotonTrackedTarget lastTarget;

  boolean goalReached  = false;

  public ChaseTagCommand(
        PhotonCamera photonCamera, 
        SwerveSubsystem drivetrainSubsystem,
        Supplier<Pose2d> poseProvider, int tagToChase) {
    this.photonCamera = photonCamera;
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.poseProvider = poseProvider;
    this.tagToChase=tagToChase;

    xController.setTolerance(0.2);
    yController.setTolerance(0.2);
    omegaController.setTolerance(Units.degreesToRadians(5));
    omegaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drivetrainSubsystem);
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

    SmartDashboard.putNumber("TagChase robotPose.X", robotPose.getX());
    SmartDashboard.putNumber("TagChase robotPose.Y", robotPose.getY());
    SmartDashboard.putNumber("TagChase robotPose.Angle", robotPose2d.getRotation().getRadians());
    var photonRes = photonCamera.getLatestResult();
    if (photonRes.hasTargets()) {
      // Find the tag we want to chase
      var targetOpt = photonRes.getTargets().stream()
          .filter(t -> t.getFiducialId() == this.tagToChase)
          .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() <= .2 && t.getPoseAmbiguity() != -1)
          .findFirst();
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
          var goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();

          // Drive
          xController.setGoal(goalPose.getX());
          yController.setGoal(goalPose.getY());
          omegaController.setGoal(goalPose.getRotation().getRadians());
          SmartDashboard.putNumber("TagChase goal Pose X", goalPose.getX());
          SmartDashboard.putNumber("TagChase goal Pose Y", goalPose.getY());
          SmartDashboard.putNumber("TagChase goal Pose Omega", goalPose.getRotation().getRadians());
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
  // @Override
  // public boolean isFinished() {
  //   return xController.atGoal() && yController.atGoal() && omegaController.atGoal();
  // };

  }
