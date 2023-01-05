package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

    private final TalonFX driveMotor;
    private final TalonFX turningMotor;

    // private final CANCoder driveEncoder;
    // private final CANCoder turningEncoder;

    private final PIDController turningPidController;

    private final CANCoder CANCoder;
    private final boolean CANCoderReversed;
    private final double CANCoderOffsetRad;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int CANCoderId, double CANCoderOffset, boolean CANCoderReversed) {

        this.CANCoderOffsetRad = CANCoderOffset;
        this.CANCoderReversed = CANCoderReversed;
        CANCoder = new CANCoder(CANCoderId);

        driveMotor = new TalonFX(driveMotorId);
        turningMotor = new TalonFX(turningMotorId);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);


        // driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        // driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        // turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        // turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public double getDrivePosition() {
        return driveMotor.getSelectedSensorPosition()*ModuleConstants.kDriveEncoderRot2Meter*ModuleConstants.kDriveEncoderTicks2Rot;
    }

    public double getTurningPosition() {
        return turningMotor.getSelectedSensorPosition() / ModuleConstants.kTurningEncoderRadians2Ticks;
    }

    public double getDriveVelocity() {
        return driveMotor.getSelectedSensorVelocity()*ModuleConstants.kDriveEncoderRPM2MeterPerSec*ModuleConstants.kDriveTicksPer100ms2RPM;
    }

    public double getTurningVelocity() {
        return turningMotor.getSelectedSensorVelocity()*ModuleConstants.kTurningEncoderRPM2RadPerSec*ModuleConstants.kDriveEncoderTicks2Rot*10*60;
    }

    public double getCANCoderRad() {
        double angle = CANCoder.getAbsolutePosition();
        angle *= 2.0 * Math.PI;
        angle -= CANCoderOffsetRad;
        return angle * (CANCoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        turningMotor.setSelectedSensorPosition(getCANCoderRad() * ModuleConstants.kTurningEncoderRadians2Ticks);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(TalonFXControlMode.Velocity, state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond * DriveConstants.kFalconMaxSetSpeed);
        turningMotor.set(TalonFXControlMode.Position, state.angle.getRadians() * ModuleConstants.kTurningEncoderRadians2Ticks);//turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        // SmartDashboard.putString("Swerve[" + CANCoder.getChannel() + "] state", state.toString());
    }

    public void stop() {
        driveMotor.set(TalonFXControlMode.PercentOutput, 0);
        turningMotor.set(TalonFXControlMode.PercentOutput, 0);
    }
}