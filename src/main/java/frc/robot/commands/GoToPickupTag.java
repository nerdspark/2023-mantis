package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OffsetFromTargetAprTag;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class GoToPickupTag extends CommandBase {

    private static final TrapezoidProfile.Constraints X_CONSTRAINTS =
            new TrapezoidProfile.Constraints(VisionConstants.MAX_VELOCITY, VisionConstants.MAX_ACCELARATION);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS =
            new TrapezoidProfile.Constraints(VisionConstants.MAX_VELOCITY, VisionConstants.MAX_ACCELARATION);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRATINTS = new TrapezoidProfile.Constraints(
            VisionConstants.MAX_VELOCITY_ROTATION, VisionConstants.MAX_ACCELARATION_ROTATION);

    private int tagToAlign;

    Pose2d centerGoalPose;
    Pose3d cameraPose;

    private static final Transform2d TAG_TO_GOAL =
            new Transform2d(new Translation2d(1.905, 1.29), Rotation2d.fromDegrees(180.0));
    private static final Transform2d TAG_TO_GOAL_BLUE =
            new Transform2d(new Translation2d(0.5, 0.5), Rotation2d.fromDegrees(180.0));
    private OffsetFromTargetAprTag offsetFromTarget = OffsetFromTargetAprTag.CENTER;
    private Transform2d GOAL_OFFSET = null;

    private PhotonCamera photonCamera;
    private final SwerveSubsystem drivetrainSubsystem;
    private final Supplier<Pose2d> poseProvider;

    private final ProfiledPIDController xController = new ProfiledPIDController(
            VisionConstants.kPXController, VisionConstants.kIXController, VisionConstants.kIXController, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(
            VisionConstants.kPYController, VisionConstants.kIYController, VisionConstants.kDYController, Y_CONSTRAINTS);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(
            VisionConstants.kPThetaController,
            VisionConstants.kIThetaController,
            VisionConstants.kDThetaController,
            OMEGA_CONSTRATINTS);

    private PhotonTrackedTarget lastTarget;
    private PhotonTrackedTarget originalTarget;

    boolean targetReached = false;

    public GoToPickupTag(SwerveSubsystem drivetrainSubsystem, Supplier<Pose2d> poseProvider) {

        this.drivetrainSubsystem = drivetrainSubsystem;
        this.poseProvider = poseProvider;

        this.GOAL_OFFSET = new Transform2d(
                new Translation2d(this.offsetFromTarget.xOffset, this.offsetFromTarget.yOffset),
                Rotation2d.fromDegrees(this.offsetFromTarget.rotationOffset));

        xController.setTolerance(VisionConstants.TRANSLATION_TOLERANCE);
        yController.setTolerance(VisionConstants.TRANSLATION_TOLERANCE);
        omegaController.setTolerance(VisionConstants.ROTATION_TOLERANCE);
        omegaController.enableContinuousInput(-Math.PI, Math.PI);

        if (DriverStation.getAlliance() == Alliance.Red) {
            tagToAlign = 5;
            photonCamera = RobotContainer.photonCamera;
            offsetFromTarget = OffsetFromTargetAprTag.PICKUPRED;
        } else {
            tagToAlign = 4;
            photonCamera = RobotContainer.photonCameraBack;
            offsetFromTarget = OffsetFromTargetAprTag.PICKUPBLUE;
        }

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        lastTarget = null;
        var robotPose = poseProvider.get();

        // SmartDashboard.putNumber("GoToPickup TagChaseInit robotPose.X", robotPose.getX());
        // SmartDashboard.putNumber("GoToPickup TagChaseInit robotPose.Y", robotPose.getY());
        // SmartDashboard.putNumber(
        //         "GoToPickup TagChaseInit robotPose.Angle",
        //         robotPose.getRotation().getRadians());
        omegaController.reset(
                robotPose.getRotation().getRadians(), drivetrainSubsystem.getChassisSpeeds().omegaRadiansPerSecond);
        xController.reset(robotPose.getX(), drivetrainSubsystem.getChassisSpeeds().vxMetersPerSecond);
        yController.reset(robotPose.getY(), drivetrainSubsystem.getChassisSpeeds().vyMetersPerSecond);

        targetReached = false;

        if (DriverStation.getAlliance() == Alliance.Red) {
            tagToAlign = 5;
            photonCamera = RobotContainer.photonCamera;
            offsetFromTarget = OffsetFromTargetAprTag.PICKUPRED;
        } else {
            tagToAlign = 4;
            photonCamera = RobotContainer.photonCameraBack;
            offsetFromTarget = OffsetFromTargetAprTag.PICKUPRED;
        }
    }

    @Override
    public void execute() {
        // SmartDashboard.putNumber("TagToAlign", tagToAlign);
        // SmartDashboard.putString("PHotoncamera", photonCamera.getName());

        var robotPose2d = poseProvider.get();

        var robotPose = new Pose3d(
                robotPose2d.getX(),
                robotPose2d.getY(),
                0.0,
                new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));

        // SmartDashboard.putNumber("GoToPickup robotPose.X", robotPose.getX());
        // SmartDashboard.putNumber("GoToPickup robotPose.Y", robotPose.getY());
        // SmartDashboard.putNumber(
        //         "GoToPickup robotPose.Angle", robotPose2d.getRotation().getRadians());
        var photonRes = photonCamera.getLatestResult();
        // SmartDashboard.putBoolean("Target Found? GoToPickup", photonRes.hasTargets());

        if (photonRes.hasTargets()) {
            // Find the tag we want to chase

            Optional<PhotonTrackedTarget> targetOpt = null;
            // if(targetFound == false) {
            targetOpt = photonRes.getTargets().stream()
                    .filter(t -> t.getFiducialId() == tagToAlign)
                    .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() <= .95 && t.getPoseAmbiguity() != -1)
                    .findFirst();
            // }

            if (targetOpt != null) {
                if (targetOpt.isPresent()) {
                    var target = targetOpt.get();
                    if (!target.equals(lastTarget)) {
                        // This is new target data, so recalculate the goal
                        lastTarget = target;

                        // Transform the robot's pose to find the camera's pose
                        if (DriverStation.getAlliance() == Alliance.Red) {
                            cameraPose =
                                    robotPose.transformBy(Constants.VisionConstants.APRILTAG_CAMERA_TO_ROBOT.inverse());
                        } else {
                            cameraPose = robotPose.transformBy(
                                    Constants.VisionConstants.APRILTAG_CAMERA_TO_ROBOT_BACK.inverse());
                        }
                        // Trasnform the camera's pose to the target's pose
                        var camToTarget = target.getBestCameraToTarget();
                        var targetPose = cameraPose.transformBy(camToTarget);

                        // Transform the tag's pose to set our goal
                        // var goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();

                        if (DriverStation.getAlliance() == Alliance.Red) {
                            centerGoalPose = targetPose.toPose2d().transformBy(TAG_TO_GOAL);
                        } else {
                            centerGoalPose = targetPose.toPose2d().transformBy(TAG_TO_GOAL_BLUE);
                        }

                        // offset the goal pose by offset from target to align to scoring location.

                        var goalPose = centerGoalPose.transformBy(GOAL_OFFSET);

                        targetReached = isRobotAtGoalPose(goalPose, robotPose2d);

                        // Drive
                        xController.setGoal(goalPose.getX());
                        yController.setGoal(goalPose.getY());
                        omegaController.setGoal(goalPose.getRotation().getRadians());
                        // SmartDashboard.putNumber("GoToPickup goal Pose X", goalPose.getX());
                        // SmartDashboard.putNumber("GoToPickup goal Pose Y", goalPose.getY());
                        // SmartDashboard.putNumber(
                        //         "GoToPickup goal Pose Omega",
                        //         goalPose.getRotation().getRadians());
                    }
                }
            }
        }

        if (lastTarget == null) {
            // No target has been visible
            drivetrainSubsystem.stopModules();
        } else {

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

            // SmartDashboard.putNumber("GoToPickup  X Speed", xSpeed);
            // SmartDashboard.putNumber("GoToPickup  Y Speed", ySpeed);
            // SmartDashboard.putNumber("GoToPickup Omega Speed", omegaSpeed);

            ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, omegaSpeed, drivetrainSubsystem.getRotation2d());
            SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

            drivetrainSubsystem.setModuleStates(moduleStates);
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.stopModules();
        // if (targetReached) {
        //     CommandScheduler.getInstance()
        //             .schedule(new BucketPickupCommand(
        //                     RobotContainer.getElevatorSubsystem(),
        //                     RobotContainer.getWristSubsystem(),
        //                     RobotContainer.getBucketSubsystem(),
        //                     RobotContainer.getArmSubsystem(),
        //                     RobotContainer.getGripperSubsystem()));
        // }
    }

    // // Returns true when the command should end.
    @Override
    public boolean isFinished() {

        // return xController.atGoal() && yController.atGoal() && omegaController.atGoal();
        return targetReached || lastTarget == null;
    }

    private boolean isRobotAtGoalPose(Pose2d goalPose, Pose2d robotPose) {

        Transform2d poseDifference = goalPose.minus(robotPose);

        // SmartDashboard.putNumber("GoToTagCommand poseDifferenceX", poseDifference.getX());
        // SmartDashboard.putNumber("GoToTagCommand poseDifference Y", poseDifference.getY());
        // SmartDashboard.putNumber(
        //         "GoToTagCommand poseDifference Angle",
        //         poseDifference.getRotation().getDegrees());

        return (Math.abs(poseDifference.getX()) <= VisionConstants.TRANSLATION_TOLERANCE)
                && (Math.abs(poseDifference.getY()) <= VisionConstants.TRANSLATION_TOLERANCE)
                && (Math.abs(poseDifference.getRotation().getDegrees()) <= VisionConstants.ROTATION_TOLERANCE);
    }
}
