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
import frc.robot.Constants.LimeLightConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.function.Supplier;

public class SwerveJoystickCmd extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;
    private final LimelightSubsystem limeLightSubSystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningTargX, turningTargY;
    private final Supplier<Boolean> fieldOrientedFunction, resetGyroButton, cancelTurn, topSpeed, rightBumper;
    private final Supplier<Integer> DPAD;
    private final Supplier<Double> leftTrigger;
    private final Supplier<Double> rightTrigger;
    private final SlewRateLimiter speedLimiter, turningLimiter;
    private final PIDController targetTurnController = new PIDController(
            DriveConstants.kPTargetTurning, DriveConstants.kITargetTurning, DriveConstants.kDTargetTurning);
    private final PIDController LimeLightController =
            new PIDController(LimeLightConstants.kP, LimeLightConstants.kI, LimeLightConstants.kD);
    private final SlewRateLimiter LimeLightLimiter = new SlewRateLimiter(LimeLightConstants.maxAccel);
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
            LimelightSubsystem limelightSubsystem,
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
            Supplier<Boolean> topSpeed,
            Supplier<Boolean> rightBumper) {
        this.swerveSubsystem = swerveSubsystem;
        this.limeLightSubSystem = limelightSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningTargX = turningTargX;
        this.turningTargY = turningTargY;
        this.DPAD = DPAD;
        this.rightTrigger = rightTrigger;
        this.leftTrigger = leftTrigger;
        this.cancelTurn = cancelTurn;
        this.topSpeed = topSpeed;
        this.rightBumper = rightBumper;
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

        swerveSubsystem.enableBrakeMode(true);
        //        targetAngle = -swerveSubsystem.getHeading() * Math.PI / 180;
        targetAngle = -(swerveSubsystem.getHeading()) * Math.PI / 180 + swerveSubsystem.getAddToTargetAngle();
        //        if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
        //            targetAngle += Math.PI;
        //        }
    }

    @Override
    public void execute() {

        // 1. Get real-time joystick inputs
        // double driveSpeed =
        // speedLimiter.calculate(OIConstants.driverMultiplier*Math.pow(Math.abs((ySpdFunction.get()*ySpdFunction.get())
        // + (xSpdFunction.get()*xSpdFunction.get())), OIConstants.driverPower/2)) *
        // DriveConstants.kTeleDriveMaxSpeedMetersPerSecond + OIConstants.driverBaseSpeedMetersPerSecond;
        double prevJoyMagnitude = joystickMagnitude;
        double prevDriveAngle = driveAngle;
        driveAngle = Math.atan2(-ySpdFunction.get(), xSpdFunction.get()) + swerveSubsystem.getAddToTargetAngle();
        joystickMagnitude = Math.sqrt((ySpdFunction.get() * ySpdFunction.get())
                + (xSpdFunction.get()
                        * xSpdFunction.get())); // Math.max(Math.abs(ySpdFunction.get()), Math.abs(xSpdFunction.get()));

        double driveSpeed = ((topSpeed.get() ? OIConstants.driverTopEXPMultiplier : OIConstants.driverEXPMultiplier)
                        * (Math.pow(Math.E, joystickMagnitude * OIConstants.driverEXPJoyMultiplier)
                                + OIConstants.driverBaseSpeed))
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
        SmartDashboard.putNumber("getAddToTargetAngle", swerveSubsystem.getAddToTargetAngle());
        double currentAngle = -(swerveSubsystem.getHeading()) * Math.PI / 180 + swerveSubsystem.getAddToTargetAngle();
        double turningSpeed = 0;

        boolean isArmOut =
                switch (RobotContainer.getArmSubsystem().getArmPositionState()) {
                    case HIGH_DROP, MID_DROP -> true;
                    default -> false;
                };

        prevDrivetrainPose3 = prevDrivetrainPose2;
        prevDrivetrainPose2 = prevDrivetrainPose;
        prevDrivetrainPose = currentDrivetrainPose;
        currentDrivetrainPose = swerveSubsystem
                .getPose()
                .getTranslation(); // Math.sqrt((swerveSubsystem.getPose().getTranslation().getX()*swerveSubsystem.getPose().getTranslation().getX())+(swerveSubsystem.getPose().getTranslation().getY()*swerveSubsystem.getPose().getTranslation().getY()));
        if (resetGyroButton.get()) {
            zeroHeading();
            swerveSubsystem.resetOdometry(new Pose2d());
        } else if (DPAD.get() != -1 && !isArmOut) {
            targetAngle = ((DPAD.get()) * Math.PI / 180d);
        }
        targetTurnController.enableContinuousInput(-Math.PI, Math.PI);
        turningSpeed = Math.abs(targetTurnController.calculate(currentAngle, targetAngle)) < 6
                ? targetTurnController.calculate(currentAngle, targetAngle)
                : 6;
        // SmartDashboard.putNumber("prevdrivetrainposeX", prevDrivetrainPose.getX());
        // SmartDashboard.putNumber("prevdrivetrainposeY", prevDrivetrainPose.getY());
        // SmartDashboard.putNumber("currdrivetrainposeX", currentDrivetrainPose.getX());
        // SmartDashboard.putNumber("currdrivetrainposeY", currentDrivetrainPose.getY());
        if (Math.sqrt(((prevDrivetrainPose3.getX() - currentDrivetrainPose.getX())
                                        * (prevDrivetrainPose3.getX() - currentDrivetrainPose.getX()))
                                + ((prevDrivetrainPose3.getY() - currentDrivetrainPose.getY())
                                        * (prevDrivetrainPose3.getY() - currentDrivetrainPose.getY())))
                        > OIConstants.kDeadbandSpeed
                && (joystickMagnitude < OIConstants.kDeadbandDrive * 3)
                && Math.abs(targetAngle - currentAngle) < DriveConstants.kTargetTurningDeadband * 3) {
            //                        turningSpeed = 0;
            //            driveAngle = prevDriveAngle;
            // SmartDashboard.putString("PID turning?", "disabled - moving fastslowing down");
        } else {
            // SmartDashboard.putString("PID turning?", "yes");
        }

        if (((Math.abs(targetAngle - currentAngle) < DriveConstants.kTargetTurningDeadband) && joystickMagnitude < 0.1)
                || cancelTurn.get()) {
            turningSpeed = 0;
            // SmartDashboard.putString("PID turning?", "deadband");
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
            // SmartDashboard.putString("joymagnitude deadband?", "on");
        } else {
            // SmartDashboard.putString("joymagnitude deadband?", "off");
        }
        double ySpeed;
        if (!topSpeed.get() && limeLightSubSystem.getX() != 0) {
            if (Math.abs(limeLightSubSystem.getX()) > 2) {
                ySpeed = (swerveSubsystem.getAddToTargetAngle() == Math.PI ? -1 : 1)
                        * ((Math.abs(LimeLightLimiter.calculate(
                                                LimeLightController.calculate(limeLightSubSystem.getX(), 0)))
                                        < LimeLightConstants.maxVel)
                                ? LimeLightLimiter.calculate(
                                        LimeLightController.calculate(limeLightSubSystem.getX(), 0))
                                : Math.copySign(
                                        LimeLightConstants.maxVel,
                                        LimeLightLimiter.calculate(
                                                LimeLightController.calculate(limeLightSubSystem.getX(), 0))));
            } else {
                ySpeed = 0;
            }
            SmartDashboard.putNumber("ySpeed", ySpeed);
            SmartDashboard.putString("limelightalign", "on");
            SmartDashboard.putNumber("limeLightSubSystem.getX()", limeLightSubSystem.getX());
        } else {
            ySpeed = (Math.sin(driveAngle) * driveSpeed);
            SmartDashboard.putString("limelightalign", "off");
            SmartDashboard.putNumber("limeLightSubSystem.getX()", limeLightSubSystem.getX());
            SmartDashboard.putNumber("ySpeed", ySpeed);
            LimeLightLimiter.reset(ySpeed);
        }

        double xSpeed = (Math.cos(driveAngle) * driveSpeed);
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
            turningSpeed =
                    -turningTargX.get() * Math.sqrt(Math.abs(turningTargX.get())) * OIConstants.joystickTurningGain;
            // SmartDashboard.putString("PID turning?", "joystickturning");
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
        turningSpeed = turningSpeed * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
        if (joystickMagnitude > OIConstants.targetTurnGainScheduleSpeed) {
            // SmartDashboard.putString("targetTurnGain", "fastGain");
            turningSpeed = turningSpeed;
        } else {
            // SmartDashboard.putString("targetTurnGain", "slowGain");
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
        if (rightBumper.get()) {
            swerveSubsystem.setWheelsToX();
        } else {
            swerveSubsystem.setModuleStates(moduleStates);
        }
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
        targetAngle = -(swerveSubsystem.getHeading()) * Math.PI / 180;
        swerveSubsystem.setAddToTargetAngle(0);

        //        if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
        //            targetAngle += Math.PI;
        //        }
    }
}
