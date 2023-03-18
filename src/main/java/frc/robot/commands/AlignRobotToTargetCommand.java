package frc.robot.commands;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.LimeLightSubSystem;
import frc.robot.subsystems.SwerveSubsystem;


public class AlignRobotToTargetCommand extends CommandBase {
    private static final double ROTATION_STATIC_CONSTANT = 0.3;
    public static final double MAX_VOLTAGE = 12.0;
    // public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0
    //         * SdsModuleConfigurations.MK4_L3.getDriveReduction() * SdsModuleConfigurations.MK4_L3.getWheelDiameter()
    //         * Math.PI;
    // public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND
    //         / Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

            public static final double MAX_VELOCITY_METERS_PER_SECOND = DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond;


    private final SwerveSubsystem drivetrain;
    private final LimeLightSubSystem vision;

    private final PIDController controller = new PIDController(5,0.,0);

    private boolean targetSeen = false;

    private final DoubleSupplier xAxis;
    private final DoubleSupplier yAxis;

    public AlignRobotToTargetCommand(SwerveSubsystem drivetrain, LimeLightSubSystem vision, DoubleSupplier xAxis,
            DoubleSupplier yAxis) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.xAxis = xAxis;
        this.yAxis = yAxis;

        controller.setTolerance(2);

        controller.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drivetrain);
    }

    public AlignRobotToTargetCommand(SwerveSubsystem drivetrain, LimeLightSubSystem vision) {
        this(drivetrain, vision, () -> 0.0, () -> 0.0);
    }

    @Override
    public void initialize() {
        controller.reset();
    }

    @Override
    public void execute() {
        if (targetSeen) {
            Rotation2d currentAngle = drivetrain.getPose().getRotation();

            controller.setSetpoint(vision.getTargetPose3d_RobotSpace().toPose2d().getRotation().getRadians());

            double rotationalVelocity = controller.calculate(currentAngle.getRadians(), Robot.kDefaultPeriod);

            // rotationalVelocity += Math.copySign(ROTATION_STATIC_CONSTANT / SwerveSubsystem.MAX_VOLTAGE
            //         * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, rotationalVelocity);

            
            rotationalVelocity += Math.copySign(ROTATION_STATIC_CONSTANT / MAX_VOLTAGE
                    * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, rotationalVelocity);

            // drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xAxis.getAsDouble(), yAxis.getAsDouble(),
            //         -rotationalVelocity, currentAngle));

            
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xAxis.getAsDouble(), yAxis.getAsDouble(), rotationalVelocity, drivetrain.getRotation2d());
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
  
        drivetrain.setModuleStates(moduleStates);    
        

        } else {
            targetSeen = vision.hasAprilTagTarget();
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopModules();
    }
}