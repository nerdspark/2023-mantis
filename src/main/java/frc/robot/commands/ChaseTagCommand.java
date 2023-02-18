package frc.robot.commands;

import static frc.robot.Constants.VisionConstants.CAMERA_TO_ROBOT;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;


public class ChaseTagCommand extends CommandBase {
  
  private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(1, 0.5);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(1, 0.5);
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRATINTS = 
      new TrapezoidProfile.Constraints(4, 4);
  
  private static final int TAG_TO_CHASE = 1;
  private static final Transform2d TAG_TO_GOAL = new Transform2d(new Translation2d(0.5, 0), Rotation2d.fromDegrees(180.0));

  private final PhotonCamera photonCamera;
  private final SwerveSubsystem drivetrainSubsystem;
  private final Supplier<Pose2d> poseProvider;

  private final ProfiledPIDController xController = new ProfiledPIDController(2.5, 0, 0, X_CONSTRAINTS);
  private final ProfiledPIDController yController = new ProfiledPIDController(2.5, 0, 0, Y_CONSTRAINTS);
  private final ProfiledPIDController omegaController = new ProfiledPIDController(1, 0, 0, OMEGA_CONSTRATINTS);

  private Pose2d goalPose;
  private PhotonTrackedTarget lastTarget;

  boolean goalReached  = false;
  int count;

  public ChaseTagCommand(
        PhotonCamera photonCamera, 
        SwerveSubsystem drivetrainSubsystem,
        Supplier<Pose2d> poseProvider) {
    this.photonCamera = photonCamera;
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.poseProvider = poseProvider;

    xController.setTolerance(0.1);
    yController.setTolerance(0.1);
    omegaController.setTolerance(Units.degreesToRadians(3));
    omegaController.enableContinuousInput(-1, 1);

    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    count=0;
    goalPose = null;
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
    var robotPose = poseProvider.get();
    SmartDashboard.putNumber("TagChase Count",count);

    SmartDashboard.putNumber("TagChase robotPose.X", robotPose.getX());
    SmartDashboard.putNumber("TagChase robotPose.Y", robotPose.getY());
    SmartDashboard.putNumber("TagChase robotPose.Angle", robotPose.getRotation().getRadians());
    var photonRes = photonCamera.getLatestResult();
    if (photonRes.hasTargets()) {
      // Find the tag we want to chase
      var targetOpt = photonRes.getTargets().stream()
          .filter(t -> t.getFiducialId() == TAG_TO_CHASE)
          .findFirst();
      if (targetOpt.isPresent() && count < 1) {
        var target = targetOpt.get();
        if (!target.equals(lastTarget)) {
          // This is new target data, so recalculate the goal
          lastTarget = target;

          count++;

          // Get the transformation from the camera to the tag (in 2d)
          var camToTarget = target.getBestCameraToTarget();

          double X = target.getBestCameraToTarget().getX();
          double Y = target.getBestCameraToTarget().getY();
          double yaw = target.getYaw();
          SmartDashboard.putNumber("TagChase Target getFiducialId", target.getFiducialId());
          SmartDashboard.putNumber("TagChase Target X", X);
          SmartDashboard.putNumber("TagChase Target Y", Y);
          SmartDashboard.putNumber("TagChase Target Yaw", yaw);


          var transform = new Transform2d(
            camToTarget.getTranslation().toTranslation2d(),
            camToTarget.getRotation().toRotation2d().minus(Rotation2d.fromDegrees(90)));
            
            // Transform the robot's pose to find the tag's pose
            var cameraPose = robotPose.transformBy(CAMERA_TO_ROBOT.inverse());
            Pose2d targetPose = cameraPose.transformBy(transform);
            
            // Transform the tag's pose to set our goal
            goalPose = targetPose.transformBy(TAG_TO_GOAL);
        }

        if (null != goalPose) {
          // Drive

          SmartDashboard.putNumber("TagChase goal Pose X", goalPose.getX());
          SmartDashboard.putNumber("TagChase goal Pose Y", goalPose.getY());
          SmartDashboard.putNumber("TagChase goal Pose Omega", goalPose.getRotation().getRadians());

    
          xController.setGoal(goalPose.getX());
          yController.setGoal(goalPose.getY());
          omegaController.setGoal(goalPose.getRotation().getRadians());
        }
      }
    }
   
    if(goalPose != null){
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

    if(xController.atGoal() && yController.atGoal() && omegaController.atGoal()){
      goalReached = true;
    }
  }
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return goalReached;

  }

}
