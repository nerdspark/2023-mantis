package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCmd extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningTargX, turningTargY;
    private final Supplier<Boolean> fieldOrientedFunction, resetGyroButton;
    private final Supplier<Integer> DPAD;
    private final Supplier<Double> leftTrigger;
    private final Supplier<Double> rightTrigger;
    private final SlewRateLimiter speedLimiter, turningLimiter;
    private final PIDController targetTurnController = new PIDController(DriveConstants.kPTargetTurning, DriveConstants.kITargetTurning, DriveConstants.kDTargetTurning);

    private  double targetAngle;

    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningTargX, Supplier<Double> turningTargY,
            Supplier<Boolean> fieldOrientedFunction, Supplier<Integer> DPAD, Supplier<Double> leftTrigger, Supplier<Double> rightTrigger, Supplier<Boolean> resetGyroButton) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningTargX = turningTargX;
        this.turningTargY = turningTargY;
        this.DPAD = DPAD;
        this.rightTrigger = rightTrigger;
        this.leftTrigger = leftTrigger;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.resetGyroButton = resetGyroButton;
        this.speedLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        // swerveSubsystem.setGains();
    }

    @Override
    public void execute() {
        // 1. Get real-time joystick inputs
        double driveAngle = Math.atan2(ySpdFunction.get(), xSpdFunction.get());
        double driveSpeed = speedLimiter.calculate(OIConstants.driverMultiplier*Math.pow(Math.abs((ySpdFunction.get()*ySpdFunction.get()) + (xSpdFunction.get()*xSpdFunction.get())), OIConstants.driverPower/2)) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond + OIConstants.driverBaseSpeedMetersPerSecond;
        double xSpeed = (Math.cos(driveAngle)*driveSpeed);
        double ySpeed = (Math.sin(driveAngle)*driveSpeed);
        double currentAngle = swerveSubsystem.getHeading()*Math.PI/180;
        if (resetGyroButton.get()) {
            zeroHeading();
        } else 
        if (DPAD.get() != -1) {
            targetAngle =  ((DPAD.get()-90) * Math.PI / 180d);
        } else 
        if ((leftTrigger.get() > OIConstants.triggerDeadband) || (rightTrigger.get() > OIConstants.triggerDeadband)) {
            targetAngle += ((rightTrigger.get() - leftTrigger.get()) * OIConstants.triggerMultiplier);
        } else 
        if ((turningTargX.get() * turningTargX.get()) + (turningTargY.get() * turningTargY.get()) > (OIConstants.kDeadbandSteer * OIConstants.kDeadbandSteer)) {
            targetAngle = -Math.atan2(-turningTargX.get(), -turningTargY.get());
            // targetAngle = ((targetAngle - currentAngle) % (2 * Math.PI)) + currentAngle;
            // if ((targetAngle - currentAngle) > Math.PI) {
            //     targetAngle -= 2 * Math.PI;
            // } else if ((targetAngle - currentAngle) < -Math.PI) {
            //     targetAngle += 2 * Math.PI;
            // }
        }
        targetTurnController.enableContinuousInput(-Math.PI, Math.PI);
        double turningSpeed = targetTurnController.calculate(currentAngle, targetAngle) ;
        if (((Math.abs(targetAngle - swerveSubsystem.getHeading()))<DriveConstants.kTargetTurningDeadband) && !SwerveSubsystem.driveTurning) {
            turningSpeed = 0;
        }
        // 2. Apply deadband
        
        if (((xSpdFunction.get() * xSpdFunction.get()) + (ySpdFunction.get() * ySpdFunction.get()) < (OIConstants.kDeadbandDrive * OIConstants.kDeadbandDrive))) {//Math.abs(xSpeed) < OIConstants.kDeadbandDrive && Math.abs(ySpeed) < OIConstants.kDeadbandDrive) {
            xSpeed = 0;
            ySpeed = 0;
            SmartDashboard.putString("in drive deadband", "yes");
        } else {
            SmartDashboard.putString("in drive deadband", "no");

        }

        // turningSpeed = Math.abs(turningTargX.get()) > OIConstants.kDeadbandSteer || Math.abs(turningTargY.get()) > OIConstants.kDeadbandSteer ? turningSpeed : 0.0;

        // 3. Make the driving smoother
        turningSpeed = turningLimiter.calculate(turningSpeed)
                * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
        if (xSpeed*xSpeed+ySpeed*ySpeed > OIConstants.targetTurnGainScheduleSpeed) {
            SmartDashboard.putString("targetTurnGain", "fastGain");
            turningSpeed = turningSpeed * 1.75;
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