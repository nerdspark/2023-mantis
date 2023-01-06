package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCmd extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningTargX, turningTargY;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private final PIDController targetTurnController = new PIDController(DriveConstants.kPTargetTurning, DriveConstants.kITargetTurning, DriveConstants.kDTargetTurning);

    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningTargX, Supplier<Double> turningTargY,
            Supplier<Boolean> fieldOrientedFunction) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningTargX = turningTargX;
        this.turningTargY = turningTargY;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // 1. Get real-time joystick inputs
        double xSpeed = (Math.abs(xSpdFunction.get())*xSpdFunction.get()*OIConstants.driverMultiplier);
        double ySpeed = (Math.abs(ySpdFunction.get())*ySpdFunction.get()*OIConstants.driverMultiplier);
        double targetAngle = -Math.atan2(-turningTargX.get(), -turningTargY.get());
        double turningSpeed = targetTurnController.calculate(swerveSubsystem.getHeading()*Math.PI/180, targetAngle) ;
        if ((Math.abs(targetAngle - swerveSubsystem.getHeading()))<0.1) {
            turningSpeed = 0;
        }
        // 2. Apply deadband
        
        if (Math.abs(xSpeed) < OIConstants.kDeadbandDrive && Math.abs(ySpeed) < OIConstants.kDeadbandDrive) {
            xSpeed = 0;
            ySpeed = 0;
        }
        turningSpeed = Math.abs(turningTargX.get()) > OIConstants.kDeadbandSteer || Math.abs(turningTargY.get()) > OIConstants.kDeadbandSteer ? turningSpeed : 0.0;

        // 3. Make the driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed)
                * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

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
}