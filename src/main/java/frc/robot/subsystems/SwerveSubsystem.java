package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveCANCoderPort,
            DriveConstants.kFrontLeftDriveCANCoderOffsetDeg,
            DriveConstants.kFrontLeftDriveCANCoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveCANCoderPort,
            DriveConstants.kFrontRightDriveCANCoderOffsetDeg,
            DriveConstants.kFrontRightDriveCANCoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveCANCoderPort,
            DriveConstants.kBackLeftDriveCANCoderOffsetDeg,
            DriveConstants.kBackLeftDriveCANCoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveCANCoderPort,
            DriveConstants.kBackRightDriveCANCoderOffsetDeg,
            DriveConstants.kBackRightDriveCANCoderReversed);
    public static boolean driveTurning = false;
    private final WPI_Pigeon2 gyro = new WPI_Pigeon2(Constants.pigeonPort, DriveConstants.canBusName);
    private final SwerveDriveOdometry odometer =
            new SwerveDriveOdometry(DriveConstants.kDriveKinematics, new Rotation2d(0), new SwerveModulePosition[] {
                frontLeft.getSwerveModulePosition(),
                frontRight.getSwerveModulePosition(),
                backLeft.getSwerveModulePosition(),
                backRight.getSwerveModulePosition()
            });

    SwerveDriveKinematics kinematics;

    public SwerveSubsystem() {
        new Thread(() -> {
                    try {
                        Thread.sleep(1000);
                        zeroHeading();
                    } catch (Exception e) {
                    }
                })
                .start();
    }

    public void zeroHeading() {
        gyro.setYaw(0);
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getYaw(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return new Pose2d(
                odometer.getPoseMeters().getTranslation(),
                new Rotation2d(odometer.getPoseMeters().getRotation().getRadians()));
        // return odometer.getPoseMeters();
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

    @Override
    public void periodic() {
        // SmartDashboard.putNumberArray("Gyro Velocity", this.getGyroVelocityXYZ());

        odometer.update(getRotation2d(), new SwerveModulePosition[] {
            frontLeft.getSwerveModulePosition(),
            frontRight.getSwerveModulePosition(),
            backLeft.getSwerveModulePosition(),
            backRight.getSwerveModulePosition()
        });

        // odometer.update(getRotation2d(), frontLeft.getState(), frontRight.getState(), backLeft.getState(),
        // backRight.getState());

        // SmartDashboard.putNumber("Robot Heading", getHeading());
        // SmartDashboard.putNumber(
        //         "Rotation2d", odometer.getPoseMeters().getRotation().getDegrees());
        // SmartDashboard.putString(
        //         "Robot Location (broken)", getPose().getTranslation().toString());
        // SmartDashboard.putNumber("X pos", odometer.getPoseMeters().getX());
        // SmartDashboard.putNumber("Y pos", odometer.getPoseMeters().getY());
        frontLeft.outputStatsSmartDashboard();
        backLeft.outputStatsSmartDashboard();
        frontRight.outputStatsSmartDashboard();
        backRight.outputStatsSmartDashboard();
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

    public double getRoll() {
        return gyro.getRoll();
    }

    public void driveSwerveDrive(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

        SwerveModuleState currenStateFL = frontLeft.getState();
        SwerveModuleState.optimize(moduleStates[0], currenStateFL.angle);
        frontLeft.setDesiredState(moduleStates[0]);

        SwerveModuleState currenStateFR = frontRight.getState();
        SwerveModuleState.optimize(moduleStates[1], currenStateFR.angle);
        frontRight.setDesiredState(moduleStates[1]);

        SwerveModuleState currenStateBL = backLeft.getState();
        SwerveModuleState.optimize(moduleStates[2], currenStateBL.angle);
        backLeft.setDesiredState(moduleStates[2]);

        SwerveModuleState currenStateBR = backRight.getState();
        SwerveModuleState.optimize(moduleStates[3], currenStateBR.angle);
        backRight.setDesiredState(moduleStates[3]);

        // for (int i = 0; i < 4; i++) {
        //     SwerveModuleState currentState = swerveModules[i].getSwerveModuleState();
        //     SwerveModuleState.optimize(moduleStates[i], currentState.angle);
        //     swerveModules[i].setSwerveModuleState(moduleStates[i]);
        // }
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        this.setModuleStates(this.kinematics.toSwerveModuleStates(chassisSpeeds));
        
    }

    public SwerveModuleState[] getModuleStates(){
        return new SwerveModuleState[] {
            this.frontLeft.getState(),
            this.frontRight.getState(),
            this.backLeft.getState(),
            this.backRight.getState()           
        };
    }

    public ChassisSpeeds getChassisSpeeds(){
        return this.kinematics.toChassisSpeeds(getModuleStates());
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

        // double error0 = Math.abs(desiredStates[0].angle.getRadians() - frontLeft.getTurningPosition())%(Math.PI);
        // error0 = error0 > 0.5*Math.PI ? error0 - Math.PI : error0;
        // double error1 = Math.abs(desiredStates[1].angle.getRadians() - frontRight.getTurningPosition())%(Math.PI);
        // error1 = error1 > 0.5*Math.PI ? error1 - Math.PI : error1;
        // double error2 = Math.abs(desiredStates[2].angle.getRadians() - backLeft.getTurningPosition())%(Math.PI);
        // error2 = error2 > 0.5*Math.PI ? error2 - Math.PI : error2;
        // double error3 = Math.abs(desiredStates[3].angle.getRadians() - backRight.getTurningPosition())%(Math.PI);
        // error3 = error3 > 0.5*Math.PI ? error3 - Math.PI : error3;
        // SmartDashboard.putString("asdf", "good");
        // if ((error0 > (DriveConstants.kEnterDriveTurningDeadband*Math.PI/180)) || (error1 >
        // (DriveConstants.kEnterDriveTurningDeadband*Math.PI/180)) || (error2 >
        // (DriveConstants.kEnterDriveTurningDeadband*Math.PI/180)) || (error3 >
        // (DriveConstants.kEnterDriveTurningDeadband*Math.PI/180)))
        // {
        //     SmartDashboard.putNumber("desiredstates", desiredStates[0].angle.getRadians());
        //     SmartDashboard.putNumber("FLpos", frontLeft.getTurningPosition());
        //     SmartDashboard.putString("asdf", "bad");
        //     desiredStates[0] = new SwerveModuleState(0, desiredStates[0].angle);
        //     desiredStates[1] = new SwerveModuleState(0, desiredStates[1].angle);
        //     desiredStates[2] = new SwerveModuleState(0, desiredStates[2].angle);
        //     desiredStates[3] = new SwerveModuleState(0, desiredStates[3].angle);
        //     driveTurning = true;
        // } else if (((error0 > (DriveConstants.kExitDriveTurningDeadband*Math.PI/180)) || (error1 >
        // (DriveConstants.kExitDriveTurningDeadband*Math.PI/180)) || (error2 >
        // (DriveConstants.kExitDriveTurningDeadband*Math.PI/180)) || (error3 >
        // (DriveConstants.kExitDriveTurningDeadband*Math.PI/180))) && !driveTurning) {
        //     driveTurning = true;
        //     desiredStates[0] = new SwerveModuleState(0, desiredStates[0].angle);
        //     desiredStates[1] = new SwerveModuleState(0, desiredStates[1].angle);
        //     desiredStates[2] = new SwerveModuleState(0, desiredStates[2].angle);
        //     desiredStates[3] = new SwerveModuleState(0, desiredStates[3].angle);
        // } else {
        driveTurning = false;
        // }

        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeft.getSwerveModulePosition(), frontRight.getSwerveModulePosition(),
            backLeft.getSwerveModulePosition(), backRight.getSwerveModulePosition()
        };
    }

    public void resetGyro() {
        zeroHeading();
    }

    // //May need to change
    // //We have to invert the angle of gyro so that rotating the robot counter-clockwise makes angle increase

    // public Rotation2d getGyroRotation(){
    //     return Rotation2d.fromDegrees(360.0 - gyro.getYaw());
    // }

    /**
     * Set the wheels to an X pattern to plant the robot.
     */
    public void setWheelsToX() {
        setModuleStates(new SwerveModuleState[] {
            // front left
            new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)),
            // front right
            new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)),
            // back left
            new SwerveModuleState(0.0, Rotation2d.fromDegrees(135.0)),
            // back right
            new SwerveModuleState(0.0, Rotation2d.fromDegrees(-135.0))
        });
    }

    /**
     * Gets the raw gyro data.
     * @return x[0], y[1], and z[2] data in degrees per second
     */
    public double[] getGyroVelocityXYZ() {
        double[] xyz = new double[3];
        gyro.getRawGyro(xyz);
        return xyz;
    }
}
