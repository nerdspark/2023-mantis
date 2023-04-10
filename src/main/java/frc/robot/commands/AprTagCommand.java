// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.ExampleSubsystem;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

/** An example command that uses an example subsystem. */
public class AprTagCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final ExampleSubsystem m_subsystem;

    public PhotonCamera photonCamera;
    private int tagToChase;
    private boolean aprTagFound = false;
    private PhotonTrackedTarget lastTarget;
    private final Supplier<Pose2d> poseProvider;
    private final double distanceFromTag;
    private final double targetHeight;

    // private static final Transform2d TAG_TO_GOAL = new Transform2d(new Translation2d(1, 0),
    // Rotation2d.fromDegrees(180.0));

    private static final Transform3d TAG_TO_GOAL =
            new Transform3d(new Translation3d(1, 0.0, 0.0), new Rotation3d(0.0, 0.0, Math.PI));
    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public AprTagCommand(
            PhotonCamera photonCamera,
            ExampleSubsystem subsystem,
            int tagToChase,
            Supplier<Pose2d> poseProvider,
            double distanceFromTag) {
        m_subsystem = subsystem;
        // SmartDashboard.putNumber("AprTagPipeLine Constructor", tagToChase);
        // Use addRequirements() here to declare subsystem dependencies.
        this.tagToChase = tagToChase;
        this.photonCamera = photonCamera;
        this.poseProvider = poseProvider;
        this.distanceFromTag = distanceFromTag;

        if (tagToChase != 4 && tagToChase != 5) {
            targetHeight = VisionConstants.GRID_APR_TAG_HEIGHT;
        } else {
            targetHeight = VisionConstants.PICKUP_APR_TAG_HEIGHT;
        }

        addRequirements(subsystem);
    }

    public AprTagCommand(
            PhotonCamera photonCamera, ExampleSubsystem subsystem, int tagToChase, Supplier<Pose2d> poseProvider) {

        this(photonCamera, subsystem, tagToChase, poseProvider, 2);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        lastTarget = null;
        this.aprTagFound = false;
        // SmartDashboard.putNumber("AprTagPipeLine INIT", this.tagToChase);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // var robotPose2d = poseProvider.get();

        // var robotPose = new Pose3d(
        //         robotPose2d.getX(),
        //         robotPose2d.getY(),
        //         0.0,
        //         new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));

        // SmartDashboard.putNumber("AprTagPipeLine RobotPose X", robotPose.getX());
        // SmartDashboard.putNumber("AprTagPipeLine RobotPose Y", robotPose.getY());
        // SmartDashboard.putNumber(
        //         "AprTagPipeLine RobotPose Angle",
        //         robotPose.getRotation().toRotation2d().getDegrees());

        var photonRes = photonCamera.getLatestResult();
        // SmartDashboard.putBoolean("Target Found?", photonRes.hasTargets());

        if (photonRes.hasTargets()) {
            // SmartDashboard.putNumber("AprTagPipeLine TargetFound", this.tagToChase);
            // Find the tag we want to chase
            var targetOpt = photonRes.getTargets().stream()
                    .filter(t -> t.getFiducialId() == this.tagToChase)
                    .findFirst();

            if (targetOpt.isPresent()) {

                var target = targetOpt.get();
                if (!target.equals(lastTarget)) {
                    // This is new target data, so recalculate the goal
                    lastTarget = target;

                    // double X = target.getBestCameraToTarget().getX();

                    // double Y = target.getBestCameraToTarget().getY();

                    // double yaw = target.getYaw();

                    double targetDistance = PhotonUtils.calculateDistanceToTargetMeters(
                            VisionConstants.CAMERA_HEIGHT_METERS,
                            targetHeight,
                            VisionConstants.CAMERA_PITCH_RADIANS,
                            0);

                    if (targetDistance <= this.distanceFromTag) this.aprTagFound = true;

                    // SmartDashboard.putNumber("AprTagPipeLine Target getFiducialId", target.getFiducialId());
                    // SmartDashboard.putNumber("AprTagPipeLine Target X", X);
                    // SmartDashboard.putNumber("AprTagPipeLine Target Y", Y);
                    // SmartDashboard.putNumber("AprTagPipeLine Target Yaw", yaw);

                    // // Get the transformation from the camera to the tag (in 2d)
                    // var camToTarget = target.getBestCameraToTarget();
                    // var transform = new Transform2d(
                    //   camToTarget.getTranslation().toTranslation2d(),
                    //   camToTarget.getRotation().toRotation2d().minus(Rotation2d.fromDegrees(90)));

                    //   // Transform the robot's pose to find the tag's pose
                    //   var cameraPose = robotPose.transformBy(Constants.VisionConstants.CAMERA_TO_ROBOT.inverse());
                    //   Pose2d targetPose = cameraPose.transformBy(transform);

                    //   // Transform the tag's pose to set our goal
                    //   goalPose = targetPose.transformBy(TAG_TO_GOAL);

                    // var cameraPose =
                    //         robotPose.transformBy(Constants.VisionConstants.APRILTAG_CAMERA_TO_ROBOT.inverse());

                    // // Trasnform the camera's pose to the target's pose
                    // var camToTarget = target.getBestCameraToTarget();
                    // var targetPose = cameraPose.transformBy(camToTarget);

                    // // Transform the tag's pose to set our goal
                    // var goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();
                    // SmartDashboard.putNumber("AprTagPipeLine goalPoseX", goalPose.getX());
                    // SmartDashboard.putNumber("AprTagPipeLine goalPoseY", goalPose.getY());
                    // SmartDashboard.putNumber(
                    //         "AprTagPipeLine goalPoseAngle",
                    //         goalPose.getRotation().getDegrees());
                }

            } else {
                // SmartDashboard.putString("No Target Tag", "Found");
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return this.aprTagFound;
    }
}
