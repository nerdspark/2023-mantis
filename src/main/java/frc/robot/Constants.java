// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.HashMap;
import java.util.Map;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }

    public static final int sensor1ID = 21;
    public static final int motorr1ID = 12;
    public static final int controllerPort = 0;
    public static final int controllerPort2 = 1;
    public static final int pigeonPort = 25;

    // JoyStick buttons
    public static final int buttonA = 1;
    public static final int buttonB = 2;
    public static final int buttonX = 3;
    public static final int buttonY = 4;
    public static final int leftBumper = 5;
    public static final int rightBumper = 6;
    public static final int back = 7;
    public static final int start = 8;
    public static final int leftStick = 9;
    public static final int rightStick = 10;

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4); // TODO: figure out right number
        // public static final double kDriveMotorGearRatio = 1d / 2048d;//TODO: figure
        // out right number
        public static final double kDriveMotorGearRatio = 1d / 6.12d;
        // public static final double kTurningMotorGearRatio = 1d / 21.429d /
        // 2048d;//TODO: figure out right number
        public static final double kTurningMotorGearRatio = 1d / (150d / 7d);
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2d * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60d;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60d;
        // public static final double kPTurning = -0.1d;
        // public static final double kTurningEncoderRadians2Ticks = 43008 / 2 /
        // Math.PI;//TODO: figure out right number
        public static final double kDriveEncoderTicks2Rot = 1d / 2048d; // TODO: figure out right number
        // public static final double kDriveTicksPer100ms2RPM = kDriveEncoderTicks2Rot *
        // 600;
        // public static final double kDriveTicks2RevsPerSecond = kDriveEncoderTicks2Rot
        // * 10;
        public static final double kTurningEncoderTicks2Rot = 1d / 2048d;
        public static final double kDriveTicks2Meters = kDriveEncoderRot2Meter * kDriveEncoderTicks2Rot;
        public static final double kDriveTicks2MeterPerSecond = kDriveEncoderTicks2Rot * 10 * kDriveEncoderRot2Meter;
        public static final double kTurnTicks2Radians = kTurningEncoderRot2Rad * kTurningEncoderTicks2Rot;
        public static final double kTurnTicks2RadiansPerSecond = kTurningEncoderTicks2Rot * 10 * kTurningEncoderRot2Rad;
    }

    public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(19);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(25);
        // Distance between front and back wheels
        public static final String canBusName = "canivore1";

        // New Kinematics
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final int kFrontLeftDriveMotorPort = 15;
        public static final int kBackLeftDriveMotorPort = 17;
        public static final int kFrontRightDriveMotorPort = 13;
        public static final int kBackRightDriveMotorPort = 11;

        public static final int kFrontLeftTurningMotorPort = 16;
        public static final int kBackLeftTurningMotorPort = 18;
        public static final int kFrontRightTurningMotorPort = 14;
        public static final int kBackRightTurningMotorPort = 12;

        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kBackLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        public static final int kFrontLeftDriveCANCoderPort = 23;
        public static final int kBackLeftDriveCANCoderPort = 24;
        public static final int kFrontRightDriveCANCoderPort = 22;
        public static final int kBackRightDriveCANCoderPort = 21;

        public static final boolean kFrontLeftDriveCANCoderReversed = true;
        public static final boolean kBackLeftDriveCANCoderReversed = true;
        public static final boolean kFrontRightDriveCANCoderReversed = true;
        public static final boolean kBackRightDriveCANCoderReversed = true;

        // Latest Kinmatics
        public static final double kFrontLeftDriveCANCoderOffsetDeg = 28 + 180; // -151.8*Math.PI/180;
        public static final double kBackLeftDriveCANCoderOffsetDeg = -127 + 180; // 53.3*Math.PI/180;
        public static final double kFrontRightDriveCANCoderOffsetDeg = 158 + 180; // (-0.66)+(-60.4*Math.PI/180);
        public static final double kBackRightDriveCANCoderOffsetDeg = -70 + 180; // 110.1*Math.PI/180;

        // Latest Kinmatics2
        // public static final double kFrontLeftDriveCANCoderOffsetRad = 25.1*Math.PI/180;
        // public static final double kBackLeftDriveCANCoderOffsetRad = 94.6*Math.PI/180;
        // public static final double kFrontRightDriveCANCoderOffsetRad = -56.9*Math.PI/180;
        // public static final double kBackRightDriveCANCoderOffsetRad = (173.6-1%80)*Math.PI/180;

        // //Old Kinematics
        // public static final double kFrontLeftDriveCANCoderOffsetRad = -6.4+180*Math.PI/180;
        // public static final double kBackLeftDriveCANCoderOffsetRad = -56.9*Math.PI/180;
        // public static final double kFrontRightDriveCANCoderOffsetRad = 25.1*Math.PI/180;
        // public static final double kBackRightDriveCANCoderOffsetRad = -265.4+360*Math.PI/180;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 3.2;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 1;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                kPhysicalMaxAngularSpeedRadiansPerSecond * 0.09;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 100;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 15;

        public static final double kFalconMaxSetSpeed = 7000d;

        public static final double kPTargetTurning = -2.6d;
        public static final double kITargetTurning = 0d;
        public static final double kDTargetTurning = -0.0d;
        public static final double kTargetTurningDeadband = 01 * Math.PI / 180;

        public static final double kRampRateTurningMotor = 0.04d;
        public static final double kPTurningMotor = 0.1d;
        public static final double kITurningMotor = 0;
        public static final double kDTurningMotor = 0;
        // public static final double kMaxSpeedTurningMotor = 100;
        public static final double kMaxAccelTurningMotor = 1;

        public static final double kRampRateDriveMotor = 0.05D;
        public static final double kPDriveMotor = 0.1d;
        public static final double kIDriveMotor = 0.0001d;
        public static final double kDDriveMotor = 2.5d;
        public static final double kFDriveMotor = 0.048d;
        public static final double kDriveMotoriZone = 0.048d;

        public static final double driveSpeedConvertMode = 4000d;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3.0d;
        public static final double kMaxAngularSpeedRadiansPerSecond =
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond;
        public static final double kMaxAccelerationMetersPerSecondSquared = 2.0d;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = 4 * Math.PI;
        public static final double kPXController = 2.5d;
        public static final double kIXController = 0d;
        public static final double kDXController = 0d;
        public static final double kPYController = 2.5d;
        public static final double kIYController = 0d;
        public static final double kDYController = 0d;
        public static final double kPThetaController = 2.5d;
        public static final double kIThetaController = 0d;
        public static final double kDThetaController = 0d;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularAccelerationRadiansPerSecondSquared);

        public static final HashMap<String, Command> autoEventMap = new HashMap<>();

        public static final double kPBalanceController = 0.015d;
        public static final double kIBalanceController = 0.0d;
        public static final double kDBalanceController = 0.01d;
        public static final double BalanceDeadBandDeg = 6.0d;
    }

    public static class ArmConstants {
        public static final int InclinovatorMotor1ID = 3; // position
        public static final int InclinovatorMotor2ID = 2; // slave to 3
        public static final int GripperMotorRID = 4; // position
        public static final int GripperMotorLID = 5; // position
        public static final int ArmMotorRID = 6; // position
        public static final int ArmMotorLID = 7; // inverted slave
        public static final int BucketMotorLID = 8; // inverted slave
        public static final int BucketMotorRID = 9; // position
        public static final int WristMotorID = 10; // motion profiling

        public static final ArmPositionData bucketPickupPosition =
                new ArmPositionData(0.0, 0.0, 1.0, -0.07, 3.0, 3.0, -10.0, -10.0, 2500.0, 4000.0);
        public static final ArmPositionData groundPickupPosition =
                new ArmPositionData(13.0, 135.0, 1.0, 0.05, 3.0, 3.0, 0.0, -12.0, 5500.0, 9000.0);
        public static final ArmPositionData shelfPickupPosition =
                new ArmPositionData(0.0, 85.0, 0.0, 0.1, 3.0, 3.0, -7.0, -7.0, 5500.0, 9000.0);
        public static final ArmPositionData highDropPosition =
                new ArmPositionData(0.0, 75.0, 18.9, 0.1, 3.0, 3.0, -15.0, 5.0, 5500.0, 9000.0);
        public static final ArmPositionData midDropPosition =
                new ArmPositionData(0.0, 80.0, 2.0, 0.1, 3.0, 3.0, -15.0, -7.0, 5500.0, 9000.0);
        public static final ArmPositionData groundDropPosition =
                new ArmPositionData(0.0, 156.0, 0.0, 0.1, -1.0, -1.0, 3.0, 3.0, 5500.0, 9000.0);
        public static final ArmPositionData homePosition =
                new ArmPositionData(0.0, 0.0, 0.25, 0.07, 3.0, 3.0, -10.0, -10.0, 2500.0, 4000.0);

        public record ArmPositionData(
                double wristCmdPos,
                double armCmdPos,
                double inclinatorCmdPos,
                double bucketCmdPos,
                double leftGripperCloseCmdPos,
                double rightGripperCloseCmdPos,
                double leftGripperOpenCmdPos,
                double rightGripperOpenCmdPos,
                double smartMotionMaxVel,
                double smartMotionMaxAccel) {}

        public static final Map<String, Double> pidConstants = new HashMap<>() {
            {
                put("kp", 0.018);
                put("max", 1.0);
                put("ki", 0.0);
                put("min", -1.0);
                put("kd", 0.002);
                put("maxI", 100.0);
                put("kiZone", 5.0);
                put("minI", -100.0);
            }
        };

        // convert the above arrays to a map, use the comments above as keys

        public static int SmartMotionMaxVel = 2500;
        public static int SmartMotionMaxAccel = 4000;
        public static double BucketInSensorPosition = 0.214286;
        public static double BucketInCommandPosition = 0.07;

        // P Gain, I Gain, D Gain, F Gain, I Zone, Min Output, Max Output
        public static final double[] GripperLGains = {0.3, 0, 0, 0, 0, -1, 1};
        public static final double[] GripperRGains = {0.3, 0, 0, 0, 0, -1, 1};
        public static final double[] WristGains = {0.4, 0, 0, 0, 0, -0.5, 0.5};
        public static final double[] BucketGains = {0.4, 0, 0, 0, 0, -0.25, 0.25};

        public static final double armAdjustMultiplier = 15.0;
        public static final double wristAdjustMultiplier = 2.0;
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kCoDriverControllerPort = 1;

        public static final int kDriverYAxis = 5;
        public static final int kDriverXAxis = 4;
        public static final int kDriverRotXAxis = 0;
        public static final int kDriverRotYAxis = 1;
        public static final int kDriverFieldOrientedButtonIdx = 2;

        public static final double kDeadbandSteer = 0.1d;
        public static final double kDeadbandDrive = 0.05d;
        public static final double kDeadbandSpeed = 0.01d;

        public static final double joystickMagnitudeChange = 0.001d;

        public static final double driverMultiplier = 0.4;
        public static final double driverTopMultiplier = 1;
        public static final double driverPower = 3.5; // 2.5 faster but clicks
        public static final double driverBaseSpeedMetersPerSecond = 00;
        public static final double triggerMultiplier = 0.1;
        public static final double triggerDeadband = 0.1;

        public static final double driverEPower = 3;
        public static final double driverEXPMultiplier = driverMultiplier * Math.pow(Math.E, -driverEPower);
        public static final double driverTopEXPMultiplier = driverTopMultiplier * Math.pow(Math.E, -driverEPower);
        public static final double driverEXPJoyMultiplier = driverEPower;
        // triggers
        public static final int kDriverLeftTrigger = 2;
        public static final int kDriverRightTrigger = 3;
        // joysticks
        public static final int kDriverRightXAxis = 4;
        public static final int kDriverRightYAxis = 5;
        public static final int kDriverLeftXAxis = 0;
        public static final int kDriverLeftYAxis = 1;
        // buttons
        public static final int kDriverButtonA = 1;
        public static final int kDriverButtonB = 2;
        public static final int kDriverButtonX = 3;
        public static final int kDriverButtonY = 4;
        public static final int kDriverLeftBumper = 5;
        public static final int kDriverRightBumper = 6;
        public static final int kDriverBackButton = 7;
        public static final int kDriverCancelTurn = 7; // back button
        public static final int kDriverTopSpeed = 5; // left bumper

        public static final double targetTurnGainScheduleSpeed = 40;

        public static final double joystickTurningGain = -8;
    }

    public static class VisionConstants {
        // public static final Transform3d robotToCam =
        // new Transform3d(
        // new Translation3d(0.2, 0.0, 0.4),
        // new Rotation3d(
        // 0, 0.524,
        // 0)); // Cam mounted facing forward, half a meter forward of center, half a
        // meter up
        // from center.
        public static final String aprTagCameraName = "OV5647";
        public static final String aprTagCameraBackName = "OV5649";
        public static final String coneCameraName = "USB_Web_Camera";

        // For color pipelines
        public static final double CAMERA_HEIGHT_METERS = 0.168;
        public static final double CONE_HEIGHT_METERS = 0.32;
        public static final double CAMERA_PITCH_RADIANS = 0;

        public static final int CUBE_PIPELINE_INDEX = 0;
        public static final int CONE_PIPELINE_INDEX = 1;

        /**
         * Physical location of the camera on the robot, relative to the center of the robot.
         */
        public static final Transform2d CAMERA_TO_ROBOT =
                new Transform2d(new Translation2d(0.051, 0.2), new Rotation2d(0.0));

        /** Physical location of the apriltag camera on the robot, relative to the center of the robot. */
        public static final Transform3d APRILTAG_CAMERA_TO_ROBOT =
                new Transform3d(new Translation3d(0.051, 0.2, -0.42), new Rotation3d(0.0, 0.0, -0.1));
        // 2, 16.5 8 inch

        public static final double FIELD_LENGTH_METERS = 16.54175;
        public static final double FIELD_WIDTH_METERS = 8.0137;

        // Pose on the opposite side of the field. Use with `relativeTo` to flip a pose to the opposite alliance
        public static final Pose2d FLIPPING_POSE =
                new Pose2d(new Translation2d(FIELD_LENGTH_METERS, FIELD_WIDTH_METERS), new Rotation2d(Math.PI));

        /** Minimum target ambiguity. Targets with higher ambiguity will be discarded */
        public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;

        // Vision Drive Constants

        public static final double TRANSLATION_TOLERANCE = 0.1; // Changed from 0.05 3/26/23
        public static final double ROTATION_TOLERANCE = 1;

        public static final double MAX_VELOCITY = 3;
        public static final double MAX_ACCELARATION = 2;
        public static final double MAX_VELOCITY_ROTATION = 1000;
        public static final double MAX_ACCELARATION_ROTATION = 1000;

        public static final double kPXController = 2.5d;
        public static final double kIXController = 0d;
        public static final double kDXController = 0d;
        public static final double kPYController = 2.5d;
        public static final double kIYController = 0d;
        public static final double kDYController = 0d;
        public static final double kPThetaController = -DriveConstants.kPTargetTurning;
        public static final double kIThetaController = 0d;
        public static final double kDThetaController = 0d;
    }

    public enum OffsetFromTargetAprTag {
        LEFT(0, 0.7452, -10),
        CENTER(0, 0.181, 0),
        RIGHT(0, -0.379, 10),
        PICKUPRED(0.5, 0.5, 0),
        PICKUPBLUE(0.5, 0.5, 0);

        public final double xOffset;
        public final double yOffset;
        public final double rotationOffset;

        private OffsetFromTargetAprTag(double xOffset, double yOffset, double rotationOffset) {
            this.xOffset = xOffset;
            this.yOffset = yOffset;
            this.rotationOffset = rotationOffset;
        }
    }
}
