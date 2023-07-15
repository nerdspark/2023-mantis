package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.MagnetFieldStrength;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

    private final TalonFX driveMotor;
    private final TalonFX turningMotor;

    private final CANCoder CANCoder;
    private final boolean CANCoderReversed;
    private final double CANCoderOffsetDeg;

    public SwerveModule(
            int driveMotorId,
            int turningMotorId,
            boolean driveMotorReversed,
            boolean turningMotorReversed,
            int CANCoderId,
            double CANCoderOffset,
            boolean CANCoderReversed) {

        this.CANCoderOffsetDeg = CANCoderOffset;
        this.CANCoderReversed = CANCoderReversed;
        CANCoder = new CANCoder(CANCoderId, DriveConstants.canBusName);

        driveMotor = new TalonFX(driveMotorId, DriveConstants.canBusName);
        turningMotor = new TalonFX(turningMotorId, DriveConstants.canBusName);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        resetEncoders();
    }

    public SwerveModulePosition getSwerveModulePosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));
    }

    public double getDrivePosition() {
        return driveMotor.getSelectedSensorPosition() * ModuleConstants.kDriveTicks2Meters;
    }

    public double getTurningPosition() {
        return turningMotor.getSelectedSensorPosition() * ModuleConstants.kTurnTicks2Radians;
    }

    public double getDriveVelocity() {
        return driveMotor.getSelectedSensorVelocity() * ModuleConstants.kDriveTicks2MeterPerSecond;
    }

    // public double getTurningVelocity() {
    //     return turningMotor.getSelectedSensorVelocity() * ModuleConstants.kTurnTicks2RadiansPerSecond;
    // }

    // public double getDriveAcceleration() {
    //     return driveMotor.getSelectedSensorVelocity() * ModuleConstants.kDriveTicks2MeterPerSecond;
    // }

    // public double getTurningAcceleration() {
    //     return turningMotor.getSelectedSensorVelocity() * ModuleConstants.kTurnTicks2RadiansPerSecond;
    // }

    public double getCANCoderRad() {
        double angle = CANCoder.getAbsolutePosition() * (CANCoderReversed ? -1.0 : 1.0);
        SmartDashboard.putNumber("pod #" + CANCoder.getDeviceID() + "CANCoder pos (degrees without offset)", angle);
        angle += CANCoderOffsetDeg;
        angle *= Math.PI / 180;
        SmartDashboard.putNumber("pod #" + CANCoder.getDeviceID() + "CANCoder pos (radians with offset)", angle);
        return angle;
    }

    public void resetEncoders() {
        driveMotor.setSelectedSensorPosition(0);
        turningMotor.setSelectedSensorPosition(getCANCoderRad() / ModuleConstants.kTurnTicks2Radians);
        // turningMotor.config_kP(1, DriveConstants.kPTurningMotor);
        // turningMotor.config_kI(1, DriveConstants.kITurningMotor);
        // turningMotor.config_kD(1, DriveConstants.kDTurningMotor);
        turningMotor.selectProfileSlot(0, 0);
        // driveMotor.config_kP(1, DriveConstants.kPDriveMotor);
        // driveMotor.config_kI(1, DriveConstants.kIDriveMotor);
        // driveMotor.config_kD(1, DriveConstants.kDDriveMotor);
        // driveMotor.config_kF(1, DriveConstants.kFDriveMotor);
        driveMotor.selectProfileSlot(0, 0);
        // turningMotor.selectProfileSlot(0, 0);
        driveMotor.configClosedloopRamp(DriveConstants.kRampRateDriveMotor);
        turningMotor.configClosedloopRamp(DriveConstants.kRampRateTurningMotor);
    }

    public void setGains() {

        turningMotor.config_kP(1, DriveConstants.kPTurningMotor);
        turningMotor.config_kI(1, DriveConstants.kITurningMotor);
        turningMotor.config_kD(1, DriveConstants.kDTurningMotor);
        turningMotor.selectProfileSlot(1, 0);
        turningMotor.setNeutralMode(NeutralMode.Brake);
        // turningMotor.configClosedloopRamp(DriveConstants.kRampRateTurningMotor);

        driveMotor.config_kP(1, DriveConstants.kPDriveMotor);
        driveMotor.config_kI(1, DriveConstants.kIDriveMotor);
        driveMotor.config_kD(1, DriveConstants.kDDriveMotor);
        driveMotor.config_kF(1, DriveConstants.kFDriveMotor);
        driveMotor.selectProfileSlot(1, 0);
        // driveMotor.configClosedloopRamp(DriveConstants.kRampRateDriveMotor);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {

        // short spin

        double target = state.angle.getRadians();
        double current = getTurningPosition();
        // target - current = pod angle error

        if (Math.abs(target - current) > (2 * Math.PI)) {
            target = ((target - current) % (2 * Math.PI)) + current;
        } // Makes sure -360 < error < 360

        if ((target - current > Math.PI)) {
            target -= (2 * Math.PI); // If error is more than 180, subtract 360
        } else if ((target - current) < -Math.PI) {
            target += (2 * Math.PI); // If error is less than -180, add 360
        }

        boolean backward = false;
        if ((target - current) > Math.PI * 0.6) {
            target -= (Math.PI);
            backward = true; // If error is more than 108, subtract 180 and drive backwards
        } else if ((target - current) < -Math.PI * 0.6) {
            target += (Math.PI);
            backward = true; // If error is less than -108, add 180 and drive backwards
        }

        // set motor commands
        driveMotor.set(
                TalonFXControlMode.PercentOutput,
                state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond * (backward ? -1 : 1));
        if (Math.abs(target - current) > 1 * Math.PI / 180) {
            turningMotor.set(TalonFXControlMode.Position, target / ModuleConstants.kTurnTicks2Radians);
        } else turningMotor.set(TalonFXControlMode.PercentOutput, 0);
        // Turning motor deadband to stop jittering

        
    }

    public void stop() {
        driveMotor.set(TalonFXControlMode.PercentOutput, 0);
    }

    public void enableBrakeMode(boolean enable) {
        if (enable) {
            driveMotor.setNeutralMode(NeutralMode.Brake);
        } else {
            driveMotor.setNeutralMode(NeutralMode.Coast);
        }
    }
}
