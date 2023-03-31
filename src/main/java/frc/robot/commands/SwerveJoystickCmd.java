package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.function.Supplier;

public class SwerveJoystickCmd extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningTargX, turningTargY;
    private final Supplier<Boolean> fieldOrientedFunction, resetGyroButton, cancelTurn, topSpeed;
    private final Supplier<Integer> DPAD;
    private final Supplier<Double> leftTrigger;
    private final Supplier<Double> rightTrigger;
    private final SlewRateLimiter speedLimiter, turningLimiter;
    private final PIDController targetTurnController = new PIDController(
            DriveConstants.kPTargetTurning, DriveConstants.kITargetTurning, DriveConstants.kDTargetTurning);

    private double targetAngle;
    private double driveAngle = 0;
    private double joystickMagnitude = 0;
    private Translation2d currentDrivetrainPose = new Translation2d();
    private Translation2d prevDrivetrainPose = new Translation2d();
    private Translation2d prevDrivetrainPose2 = new Translation2d();
    private Translation2d prevDrivetrainPose3 = new Translation2d();
    private boolean correcting = true;

    public SwerveJoystickCmd(
            SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction,
            Supplier<Double> ySpdFunction,
            Supplier<Double> turningTargX,
            Supplier<Double> turningTargY,
            Supplier<Boolean> fieldOrientedFunction,
            Supplier<Integer> DPAD,
            Supplier<Double> leftTrigger,
            Supplier<Double> rightTrigger,
            Supplier<Boolean> resetGyroButton,
            Supplier<Boolean> cancelTurn,
            Supplier<Boolean> topSpeed) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningTargX = turningTargX;
        this.turningTargY = turningTargY;
        this.DPAD = DPAD;
        this.rightTrigger = rightTrigger;
        this.leftTrigger = leftTrigger;
        this.cancelTurn = cancelTurn;
        this.topSpeed = topSpeed;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.resetGyroButton = resetGyroButton;
        this.speedLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        // swerveSubsystem.setGains();
        // zeroHeading();
        targetAngle = -swerveSubsystem.getHeading() * Math.PI / 180;
    }

    @Override
    public void execute() {

        double prevJoyMagnitude = joystickMagnitude;
        double prevDriveAngle = driveAngle;
        // 1. Get real-time joystick inputs
        // double driveSpeed =
        // speedLimiter.calculate(OIConstants.driverMultiplier*Math.pow(Math.abs((ySpdFunction.get()*ySpdFunction.get())
        // + (xSpdFunction.get()*xSpdFunction.get())), OIConstants.driverPower/2)) *
        // DriveConstants.kTeleDriveMaxSpeedMetersPerSecond + OIConstants.driverBaseSpeedMetersPerSecond;

        driveAngle = Math.atan2(-ySpdFunction.get(), xSpdFunction.get());
        joystickMagnitude = Math.abs(ySpdFunction.get()) > Math.abs(xSpdFunction.get())
                ? Math.abs(ySpdFunction.get())
                : Math.abs(xSpdFunction.get());

        double driveSpeed = (topSpeed.get() ? OIConstants.driverTopEXPMultiplier : OIConstants.driverEXPMultiplier)
                * Math.pow(Math.E, joystickMagnitude * OIConstants.driverEXPJoyMultiplier)
                * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;

        // double xSpeed =
        // OIConstants.driverMultiplier*xSpdFunction.get()*DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        // double ySpeed =
        // OIConstants.driverMultiplier*ySpdFunction.get()*DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        // double driveAngle = Math.atan2(ySpdFunction.get(), xSpdFunction.get());
        // double driveSpeed =
        // speedLimiter.calculate(OIConstants.driverMultiplier*Math.abs((ySpdFunction.get()*ySpdFunction.get()) +
        // (xSpdFunction.get()*xSpdFunction.get()))) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        // double xSpeed = (Math.cos(driveAngle)*driveSpeed);
        // double ySpeed = (Math.sin(driveAngle)*driveSpeed);
        double currentAngle = -swerveSubsystem.getHeading() * Math.PI / 180;
        double turningSpeed = 0;
        prevDrivetrainPose3 = prevDrivetrainPose2;
        prevDrivetrainPose2 = prevDrivetrainPose;
        prevDrivetrainPose = currentDrivetrainPose;
        currentDrivetrainPose = swerveSubsystem
                .getPose()
                .getTranslation(); // Math.sqrt((swerveSubsystem.getPose().getTranslation().getX()*swerveSubsystem.getPose().getTranslation().getX())+(swerveSubsystem.getPose().getTranslation().getY()*swerveSubsystem.getPose().getTranslation().getY()));
        if (resetGyroButton.get()) {
            zeroHeading();
            swerveSubsystem.resetOdometry(new Pose2d());
        } else if (DPAD.get() != -1) {
            targetAngle = ((DPAD.get()) * Math.PI / 180d);
        }
        targetTurnController.enableContinuousInput(-Math.PI, Math.PI);
        turningSpeed = targetTurnController.calculate(currentAngle, targetAngle);
        SmartDashboard.putNumber("prevdrivetrainposeX", prevDrivetrainPose.getX());
        SmartDashboard.putNumber("prevdrivetrainposeY", prevDrivetrainPose.getY());
        SmartDashboard.putNumber("currdrivetrainposeX", currentDrivetrainPose.getX());
        SmartDashboard.putNumber("currdrivetrainposeY", currentDrivetrainPose.getY());
        if (Math.sqrt(((prevDrivetrainPose3.getX() - currentDrivetrainPose.getX())
                                        * (prevDrivetrainPose3.getX() - currentDrivetrainPose.getX()))
                                + ((prevDrivetrainPose3.getY() - currentDrivetrainPose.getY())
                                        * (prevDrivetrainPose3.getY() - currentDrivetrainPose.getY())))
                        > OIConstants.kDeadbandSpeed
                && (joystickMagnitude < OIConstants.kDeadbandDrive * 3)
                && (Math.abs(currentAngle - targetAngle) < 5*Math.PI/180)) {
            turningSpeed = 0;
            driveAngle = prevDriveAngle;
            SmartDashboard.putString("PID turning?", "disabled - moving fastslowing down");
        } else {
            SmartDashboard.putString("PID turning?", "yes");
        }

        boolean isArmOut = switch(RobotContainer.getArmSubsystem().getArmPositionState()) {
            case HIGH_DROP, MID_DROP -> true;
            default -> false;
        };

        if ((Math.abs(targetAngle - currentAngle) < DriveConstants.kTargetTurningDeadband) || cancelTurn.get() || isArmOut) {
            turningSpeed = 0;
            SmartDashboard.putString("PID turning?", "deadband");
        }

        // if (Math.abs(targetAngle - currentAngle) < DriveConstants.kTargetTurningDeadband * 10
        //         && (joystickMagnitude < OIConstants.kDeadbandDrive * 2) && !correcting) {
        //     turningSpeed = 0;
        //     driveAngle = prevDriveAngle;
        //     SmartDashboard.putString("PID turning?", "stopped deadband");
        // } else if (correcting) {
        //     if (Math.abs(targetAngle - currentAngle) < DriveConstants.kTargetTurningDeadband*3) {
        //         correcting = false;
        //         SmartDashboard.putString("correcting?", "false");
        //     }
        //     SmartDashboard.putString("correcting?", "true");
        // } else if (Math.abs(targetAngle - currentAngle) > DriveConstants.kTargetTurningDeadband * 10) {
        //     correcting = true;
        //     SmartDashboard.putString("correcting?", "true");
        // }
        // if (prevJoyMagnitude - joystickMagnitude > OIConstants.joystickMagnitudeChange) {
        //     driveAngle = prevDriveAngle;
        //     SmartDashboard.putString("joymagnitude test?", "on");
        // } else {
        // SmartDashboard.putString("joymagnitude test?", "off"); }
        if (joystickMagnitude < OIConstants.kDeadbandDrive) {
            driveAngle = prevDriveAngle;
            driveSpeed = 0;
            SmartDashboard.putString("joymagnitude deadband?", "on");
        } else {
            SmartDashboard.putString("joymagnitude deadband?", "off");
        }
        double xSpeed = (Math.cos(driveAngle) * driveSpeed);
        double ySpeed = (Math.sin(driveAngle) * driveSpeed);
        // if ((turningTargX.get()*turningTargX.get()) > OIConstants.kDeadbandSteer ||
        // (turningTargY.get()*turningTargY.get()) > OIConstants.kDeadbandSteer) {
        //     targetAngle = Math.atan2(-turningTargX.get(), turningTargY.get());
        // }
        // if ((leftTrigger.get() > OIConstants.triggerDeadband) || (rightTrigger.get() > OIConstants.triggerDeadband))
        // {
        //     // targetAngle += ((rightTrigger.get() - leftTrigger.get()) * OIConstants.triggerMultiplier);
        // } else
        // if ((turningTargX.get() * turningTargX.get()) + (turningTargY.get() * turningTargY.get()) >
        // (OIConstants.kDeadbandSteer * OIConstants.kDeadbandSteer)) {
        //     targetAngle = -Math.atan2(-turningTargX.get(), -turningTargY.get());
        // }

        if (Math.abs(turningTargX.get()) > OIConstants.kDeadbandSteer) {
            targetAngle = currentAngle;
            turningSpeed = -turningTargX.get() * Math.abs(turningTargX.get()) * OIConstants.joystickTurningGain;
            SmartDashboard.putString("PID turning?", "joystickturning");
        }

        // 2. Apply deadband

        // if (/*Math.abs(xSpdFunction.get() * xSpdFunction.get()) + Math.abs(ySpdFunction.get() * ySpdFunction.get()) <
        // (OIConstants.kDeadbandDrive * OIConstants.kDeadbandDrive)) {*/Math.abs(xSpdFunction.get()) <
        // OIConstants.kDeadbandDrive && Math.abs(ySpdFunction.get()) < OIConstants.kDeadbandDrive) {
        //     xSpeed = 0;
        //     ySpeed = 0;
        //     SmartDashboard.putString("in drive deadband", "yes");
        // } else {
        //     SmartDashboard.putString("in drive deadband", "no");

        // turningSpeed = Math.abs(turningTargX.get()) > OIConstants.kDeadbandSteer || Math.abs(turningTargY.get()) >
        // OIConstants.kDeadbandSteer ? turningSpeed : 0.0;

        // 3. Make the driving smoother
        turningSpeed *= // = turningLimiter.calculate(turningSpeed) *
                DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
        if ((xSpeed * xSpeed) + (ySpeed * ySpeed) > OIConstants.targetTurnGainScheduleSpeed) {
            SmartDashboard.putString("targetTurnGain", "fastGain");
            turningSpeed = turningSpeed * 1;
        } else {
            SmartDashboard.putString("targetTurnGain", "slowGain");
        }
        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        if (fieldOrientedFunction.get()) {
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
        } else {
            // Relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public void zeroHeading() {
        swerveSubsystem.zeroHeading();
        targetAngle = 0;
    }
}
