package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftCANCoderPort,
            DriveConstants.kFrontLeftCANCoderOffsetDeg,
            DriveConstants.kFrontLeftCANCoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightCANCoderPort,
            DriveConstants.kFrontRightCANCoderOffsetDeg,
            DriveConstants.kFrontRightCANCoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftCANCoderPort,
            DriveConstants.kBackLeftCANCoderOffsetDeg,
            DriveConstants.kBackLeftCANCoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightCANCoderPort,
            DriveConstants.kBackRightCANCoderOffsetDeg,
            DriveConstants.kBackRightCANCoderReversed);

    private final WPI_Pigeon2 gyro = new WPI_Pigeon2(Constants.pigeonPort, DriveConstants.canBusName);

    private final SwerveDriveOdometry odometer =
            new SwerveDriveOdometry(DriveConstants.kDriveKinematics, new Rotation2d(0), new SwerveModulePosition[] {
                frontLeft.getSwerveModulePosition(),
                frontRight.getSwerveModulePosition(),
                backLeft.getSwerveModulePosition(),
                backRight.getSwerveModulePosition()
            });

    SwerveDriveKinematics kinematics;

    private final Field2d fieldSim = new Field2d();
    private double robotAngleOffset = 0; // robot can have 'forward' set at different angles

    public SwerveSubsystem() {
        new Thread(() -> {
                    try {
                        Thread.sleep(1000);
                        zeroHeading();
                    } catch (Exception e) {
                    }
                })
                .start();

        Shuffleboard.getTab("Autonomous").add(fieldSim);
    }

    /**
     * returns gyro angle
     */
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(Math.IEEEremainder(gyro.getYaw(), 360));
    }
    /**
     * returns odometry position and angle
     */
    public Pose2d getPose() {
        return new Pose2d(
                odometer.getPoseMeters().getTranslation(),
                new Rotation2d(odometer.getPoseMeters().getRotation().getRadians()));
    }

    /**
     * returns gyro forward tilt
     */
    public double getRoll() {
        return gyro.getRoll();
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeft.getSwerveModulePosition(), frontRight.getSwerveModulePosition(),
            backLeft.getSwerveModulePosition(), backRight.getSwerveModulePosition()
        };
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(
                getRotation2d(),
                new SwerveModulePosition[] {
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition()
                },
                pose);
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setGains() {
        frontLeft.setGains();
        frontRight.setGains();
        backLeft.setGains();
        backRight.setGains();
    }

    public void enableBrakeMode(boolean enable) {
        frontLeft.enableBrakeMode(enable);
        frontRight.enableBrakeMode(enable);
        backLeft.enableBrakeMode(enable);
        backRight.enableBrakeMode(enable);
    }

    public void zeroHeading() {
        gyro.setYaw(0);
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    /**
     * Set the wheels to an X pattern to plant the robot.
     */
    public void setWheelsToX() {
        setModuleStates(new SwerveModuleState[] {
            new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)),
            new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)),
            new SwerveModuleState(0.0, Rotation2d.fromDegrees(135.0)),
            new SwerveModuleState(0.0, Rotation2d.fromDegrees(-135.0))
        });
    }

    @Override
    public void periodic() {

        odometer.update(getRotation2d(), new SwerveModulePosition[] {
            frontLeft.getSwerveModulePosition(),
            frontRight.getSwerveModulePosition(),
            backLeft.getSwerveModulePosition(),
            backRight.getSwerveModulePosition()
        });

        fieldSim.getObject("Odometer Pos").setPose(getPose());
        fieldSim.setRobotPose(getPose());

        SmartDashboard.putNumber("Robot Gyro Heading", getHeading());
    }

    public void setRobotAngleOffset(double val) {
        this.robotAngleOffset = val;
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getYaw(), 360);
    }

    public double getRobotAngleOffset() {
        SmartDashboard.putNumber("getRobotAngleOffset", this.robotAngleOffset);
        return this.robotAngleOffset + Math.PI;
    }
}
