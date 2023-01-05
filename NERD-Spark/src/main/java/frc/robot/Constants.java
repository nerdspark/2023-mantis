// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final int sensor1ID = 21;
    public static final int motorr1ID = 12;
    public static final int controllerPort = 0;
    public static final int buttonA = 1;
    public static final int pigeonPort = 25;

    public static final int buttonY = 4;


    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);//TODO: figure out right number
        public static final double kDriveMotorGearRatio = 1 / 2048;//TODO: figure out right number
        public static final double kTurningMotorGearRatio = 1 / 21.429 / 2048;//TODO: figure out right number
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.5;

        // public static final double kTurningEncoderRadians2Ticks = 43008 / 2 / Math.PI;//TODO: figure out right number
        // public static final double kDriveEncoderTicks2Rot = 1/2048; //TODO: figure out right number
        // public static final double kDriveTicksPer100ms2RPM = kDriveEncoderTicks2Rot * 600;
    }

    public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(17);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(17);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

        public static final int kFrontLeftDriveMotorPort = 15;
        public static final int kBackLeftDriveMotorPort = 17;
        public static final int kFrontRightDriveMotorPort = 13;
        public static final int kBackRightDriveMotorPort = 11;

        public static final int kFrontLeftTurningMotorPort = 16;
        public static final int kBackLeftTurningMotorPort = 18;
        public static final int kFrontRightTurningMotorPort = 14;
        public static final int kBackRightTurningMotorPort = 12;

        public static final boolean kFrontLeftTurningEncoderReversed = false;
        public static final boolean kBackLeftTurningEncoderReversed = false;
        public static final boolean kFrontRightTurningEncoderReversed = false;
        public static final boolean kBackRightTurningEncoderReversed = false;

        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kBackLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        public static final int kFrontLeftDriveCANCoderPort = 23;
        public static final int kBackLeftDriveCANCoderPort = 24;
        public static final int kFrontRightDriveCANCoderPort = 22;
        public static final int kBackRightDriveCANCoderPort = 21;

        public static final boolean kFrontLeftDriveCANCoderReversed = false;
        public static final boolean kBackLeftDriveCANCoderReversed = false;
        public static final boolean kFrontRightDriveCANCoderReversed = false;
        public static final boolean kBackRightDriveCANCoderReversed = false;

        public static final double kFrontLeftDriveCANCoderOffsetRad = 25.1*Math.PI/180;
        public static final double kBackLeftDriveCANCoderOffsetRad = 94.6*Math.PI/180;
        public static final double kFrontRightDriveCANCoderOffsetRad = -56.9*Math.PI/180;
        public static final double kBackRightDriveCANCoderOffsetRad = -6.4*Math.PI/180;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;


        public static final double kFalconMaxSetSpeed = 50000;

        public static final double kPTargetTurning = 1;
        public static final double kITargetTurning = 0;
        public static final double kDTargetTurning = 0;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotXAxis = 4;
        public static final int kDriverRotYAxis = 3;
        public static final int kDriverFieldOrientedButtonIdx = 0;

        public static final double kDeadband = 0.05;
    }
}
