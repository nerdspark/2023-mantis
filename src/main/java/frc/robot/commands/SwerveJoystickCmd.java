package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants;
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

    private double turningSpeed = 0;

    private static final boolean turningModeTarget = false; // false = normal joystickturn, true = target turning
    private static final boolean joystickFunctionCircle =
            true; // true = circle, false = square (Speed function will be different based on max value of joystick or
    // hypotenuse)

    private double prevCurrentAngle = 0;

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
        targetAngle = -(swerveSubsystem.getHeading()) * Math.PI / 180 + swerveSubsystem.getRobotAngleOffset();
    }

    @Override
    public void execute() {

        // save previous drive settings for drift function
        double prevJoyMagnitude = joystickMagnitude;
        double prevDriveAngle = driveAngle;

        // save previous drivetrain translations for drift function
        prevDrivetrainPose3 = prevDrivetrainPose2;
        prevDrivetrainPose2 = prevDrivetrainPose;
        prevDrivetrainPose = currentDrivetrainPose;
        currentDrivetrainPose = swerveSubsystem.getPose().getTranslation();

        targetTurnController.enableContinuousInput(-Math.PI, Math.PI);

        // Create joystick angle and magnitude

        driveAngle = Math.atan2(-ySpdFunction.get(), xSpdFunction.get()) + swerveSubsystem.getRobotAngleOffset();
        joystickMagnitude = joystickFunctionCircle
                ? (Math.sqrt((ySpdFunction.get() * ySpdFunction.get()) + (xSpdFunction.get() * xSpdFunction.get())))
                : (Math.max(Math.abs(ySpdFunction.get()), Math.abs(xSpdFunction.get())));

        // Use exponential ramp, speedy button, and maximum drive speed to calculate chassis translational speed
        double driveSpeed = (joystickMagnitude * (topSpeed.get() ? OIConstants.driverTopMultiplier: OIConstants.driverMultiplier)) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        // ((topSpeed.get() ? OIConstants.driverTopEXPMultiplier : OIConstants.driverEXPMultiplier)
        //                 * (Math.pow(Math.E, joystickMagnitude * OIConstants.driverEXPJoyMultiplier)
        //                         + OIConstants.driverBaseSpeed))
        //         * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;

        // find current gyro angle, invert, convert to radians, apply offset if Auton backwards
        SmartDashboard.putNumber("getRobotAngleOffset", swerveSubsystem.getRobotAngleOffset());
        double currentAngle = -(swerveSubsystem.getHeading()) * Math.PI / 180 + swerveSubsystem.getRobotAngleOffset();

        boolean isArmOut =
                switch (RobotContainer.getArmSubsystem().getArmPositionState()) {
                    case HIGH_DROP, MID_DROP -> true;
                    default -> false;
                };

        // reset gyro function
        if (resetGyroButton.get()) {
            zeroHeading();
            swerveSubsystem.resetOdometry(new Pose2d());
            // DPAD will only point to cardinal directions and when arm is in
        } else if (DPAD.get() != -1 && !isArmOut) {
            double dpadangle =
                    switch (DPAD.get()) {
                        case 45, 315 -> 0;
                        case 135, 225 -> 180;
                        default -> DPAD.get();
                    };
            targetAngle = ((dpadangle) * Math.PI / 180d);
            // switch between stick turning modes
        } else if (turningModeTarget) {
            if ((turningTargX.get() * turningTargX.get()) + (turningTargY.get() * turningTargY.get())
                    > (OIConstants.kDeadbandSteer * OIConstants.kDeadbandSteer)) {
                targetAngle = Math.atan2(-turningTargX.get(), turningTargY.get());
            }
        } else {
            if (Math.abs(turningTargX.get()) > OIConstants.kDeadbandSteer) {
                targetAngle = currentAngle;
                prevCurrentAngle = currentAngle;
                turningSpeed =
                        -turningTargX.get() * Math.sqrt(Math.abs(turningTargX.get())) * OIConstants.joystickTurningGain;
            } else if (prevCurrentAngle != 0) {
                targetAngle += 1 * (currentAngle - prevCurrentAngle);
                prevCurrentAngle = 0;
            }
        }

        // PID turn is on if: ((not manual turn) OR DPAD is down)
        if (((turningModeTarget || (Math.abs(turningTargX.get()) < OIConstants.kDeadbandSteer)) || DPAD.get() != -1)) {
            turningSpeed = targetTurnController.calculate(currentAngle, targetAngle);
        }

        // turn disable if: in turn deadband and not moving but also not manual turning OR cancelturnbutton
        if (((Math.abs(targetAngle - currentAngle) < DriveConstants.kTargetTurningDeadband)
                        && joystickMagnitude < 0.1
                        && (turningModeTarget || (Math.abs(turningTargX.get()) < OIConstants.kDeadbandSteer)))
                || cancelTurn.get()) {
            turningSpeed = 0;
        }

        // joystick deadband and slight drift
        if (joystickMagnitude < OIConstants.kDeadbandDrive) {
            driveAngle = prevDriveAngle;
            driveSpeed = 0;
        }

        // create Y direction speed (field-oriented) and with LimeLight
        double ySpeed = 0;
        if (!topSpeed.get() && limeLightSubSystem.getX() != 0) {
            // LimeLight deadband
            if (Math.abs(limeLightSubSystem.getX()) > 2) {
                ySpeed = (swerveSubsystem.getRobotAngleOffset() == Math.PI ? -1 : 1)
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

        // create X direction speed (robot-oriented)
        double xSpeed = (Math.cos(driveAngle) * driveSpeed);

        // scale turning speed, DISABLED - schedule turning gain based on traverse speed
        turningSpeed = turningSpeed * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
        if (joystickMagnitude > OIConstants.targetTurnGainScheduleSpeed) {
            turningSpeed = turningSpeed;
        }

        // turn speeds robot-oriented and into individual modules, output to swerveSubsystem

        ChassisSpeeds chassisSpeeds;
        if (fieldOrientedFunction.get()) { // always true except when driver presses button #2
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
        } else {
            // Relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        if (rightBumper.get() || DriverStation.isAutonomous()) {
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
        swerveSubsystem.setRobotAngleOffset(Math.PI);

        //        if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
        //            targetAngle += Math.PI;
        //        }
    }
}
