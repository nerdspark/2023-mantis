// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotContainer;

/** An example command that uses an example subsystem. */
public class DriveFollowPath extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    //   private final SwerveSubsystem swerveSubsystem;
    private final PathPlannerTrajectory trajectory;

    private final PPHolonomicDriveController swerveController;
    private final boolean resetOdometry;

    private final boolean brakeMode;
    private final String pathName;
    public final Timer timer = new Timer();

    public DriveFollowPath(String pathName) {
        this(
                pathName,
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared,
                true,
                true);
    }

    public DriveFollowPath(String pathName, double maxVel, double maxAccel) {
        this(pathName, maxVel, maxAccel, true, true);
    }

    public DriveFollowPath(String pathName, double maxVel, double maxAccel, boolean resetOdometry) {
        this(pathName, maxVel, maxAccel, resetOdometry, true);
    }

    public DriveFollowPath(String pathName, double maxVel, double maxAccel, boolean resetOdometry, boolean brakeMode) {
        this.brakeMode = brakeMode;
        addRequirements(RobotContainer.getSwerveSubsystem());

        this.trajectory = PathPlanner.loadPath(pathName, maxVel, maxAccel);

        AutoConstants.autoEventMap.put("Marker1", new PrintCommand("Passed Marker1"));
        // AutoConstants.autoEventMap.put("Marker2", new PrintCommand("Passed Marker2"));

        PIDController xController = new PIDController(
                AutoConstants.kPXController, AutoConstants.kIXController, AutoConstants.kDXController);
        PIDController yController = new PIDController(
                AutoConstants.kPYController, AutoConstants.kIYController, AutoConstants.kDYController);
        // ProfiledPIDController thetaController = new ProfiledPIDController(
        //           AutoConstants.kPThetaController, AutoConstants.kIThetaController, AutoConstants.kDThetaController,
        // AutoConstants.kThetaControllerConstraints);
        PIDController thetaController = new PIDController(
                AutoConstants.kPThetaController, AutoConstants.kIThetaController, AutoConstants.kDThetaController);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        this.swerveController = new PPHolonomicDriveController(xController, yController, thetaController);
        this.resetOdometry = resetOdometry;

        this.pathName = pathName;
    }

    // /**
    //  * Creates a new ExampleCommand.
    //  *
    //  * @param subsystem The subsystem used by this command.
    //  */
    // public DriveFollowPath(ExampleSubsystem subsystem) {
    //   m_subsystem = subsystem;
    //   // Use addRequirements() here to declare subsystem dependencies.
    //   addRequirements(subsystem);
    // }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        RobotContainer.getSwerveSubsystem().enableBrakeMode(this.brakeMode);
        timer.reset();
        timer.start();
        Pose2d initialPose = trajectory.getInitialHolonomicPose();
        // if(resetOdometry) RobotContainer.getSwerveSubsystem().resetOdometry(new Pose2d(initialPose.getTranslation(),
        //  RobotContainer.getSwerveSubsystem().getRotation2d()));
        if (resetOdometry) RobotContainer.getSwerveSubsystem().resetOdometry(initialPose);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double time = timer.get();
        PathPlannerState desiredState = (PathPlannerState) trajectory.sample(time);
        ChassisSpeeds targetSpeeds =
                swerveController.calculate(RobotContainer.getSwerveSubsystem().getPose(), desiredState);
        // ChassisSpeeds targetSpeeds = swerveController.calculate(RobotContainer.getSwerveSubsystem().getPose(),
        // desiredState,
        //  new Rotation2d(desiredState.holonomicRotation.getRadians()));

        // targetSpeeds.vyMetersPerSecond = -targetSpeeds.vyMetersPerSecond;
        // targetSpeeds.omegaRadiansPerSecond = -targetSpeeds.omegaRadiansPerSecond;

        Pose2d currentPose = RobotContainer.getSwerveSubsystem().getPose();
        String tString = " [" + Math.round(timer.get() * 100) / 100.0 + "]";
        System.out.println(pathName + tString + " x error: " + (desiredState.poseMeters.getX() - currentPose.getX()));
        System.out.println(pathName + tString + " y error: " + (desiredState.poseMeters.getY() - currentPose.getY()));
        System.out.println(pathName + tString + " r error: "
                + (desiredState.holonomicRotation.getDegrees()
                        - currentPose.getRotation().getDegrees()));

        // RobotContainer.getSwerveSubsystem().driveSwerveDrive(targetSpeeds);

        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
        RobotContainer.getSwerveSubsystem().setModuleStates(moduleStates);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        timer.stop();
        RobotContainer.getSwerveSubsystem().stopModules();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }
}
