// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.CoordinateAxis;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
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
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  
  public static final int sensor1ID = 21;
  public static final int motorr1ID = 12;
  public static final int controllerPort = 0;
  public static final int controllerPort2 = 1;
  public static final int pigeonPort = 25;


  public static final class ModuleConstants {
      public static final double kWheelDiameterMeters = Units.inchesToMeters(4);//TODO: figure out right number
      //public static final double kDriveMotorGearRatio = 1d / 2048d;//TODO: figure out right number
      public static final double kDriveMotorGearRatio = 1d / 6.12d;
      //public static final double kTurningMotorGearRatio = 1d / 21.429d / 2048d;//TODO: figure out right number
      public static final double kTurningMotorGearRatio = 1d / (150d / 7d);
      public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
      public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2d * Math.PI;
      public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60d;
      public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60d;
      // public static final double kPTurning = -0.1d;
      // public static final double kTurningEncoderRadians2Ticks = 43008 / 2 / Math.PI;//TODO: figure out right number
      public static final double kDriveEncoderTicks2Rot = 1d / 2048d; //TODO: figure out right number
      // public static final double kDriveTicksPer100ms2RPM = kDriveEncoderTicks2Rot * 600;
      // public static final double kDriveTicks2RevsPerSecond = kDriveEncoderTicks2Rot * 10;
      public static final double kTurningEncoderTicks2Rot = 1d / 2048d;
      public static final double kDriveTicks2Meters = kDriveEncoderRot2Meter * kDriveEncoderTicks2Rot;
      public static final double kDriveTicks2MeterPerSecond = kDriveEncoderTicks2Rot * 10 * kDriveEncoderRot2Meter;
      public static final double kTurnTicks2Radians = kTurningEncoderRot2Rad * kTurningEncoderTicks2Rot;
      public static final double kTurnTicks2RadiansPerSecond = kTurningEncoderTicks2Rot * 10 * kTurningEncoderRot2Rad;
  }

  public static final class DriveConstants {

      public static final double kTrackWidth = Units.inchesToMeters(17);
      // Distance between right and left wheels
      public static final double kWheelBase = Units.inchesToMeters(17);
      // Distance between front and back wheels

      //Old Kinematics
      // public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      //         new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      //         new Translation2d(kWheelBase / 2, kTrackWidth / 2),
      //         new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
      //         new Translation2d(-kWheelBase / 2, +kTrackWidth / 2));

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
      public static final boolean kBackLeftDriveCANCoderReversed = false;
      public static final boolean kFrontRightDriveCANCoderReversed = false;
      public static final boolean kBackRightDriveCANCoderReversed = false;

      //Latest Kinmatics
      public static final double kFrontLeftDriveCANCoderOffsetRad = -2.72+ Math.PI;//(25.1*Math.PI/180)+1.02;
      public static final double kBackLeftDriveCANCoderOffsetRad = 94.6*Math.PI/180;
      public static final double kFrontRightDriveCANCoderOffsetRad = -56.9*Math.PI/180;
      public static final double kBackRightDriveCANCoderOffsetRad = 173.6*Math.PI/180;

      //Latest Kinmatics2
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

      public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond/1;
      public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
              kPhysicalMaxAngularSpeedRadiansPerSecond *0.09;
      public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 20;
      public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 15;


      public static final double kFalconMaxSetSpeed = 10000d;

      public static final double kPTargetTurning = -2.5d;
      public static final double kITargetTurning = 0d;    
      public static final double kDTargetTurning = 0d;
      public static final double kTargetTurningDeadband = 1*Math.PI/180;

      public static final double kRampRateTurningMotor = 0.25d;
      public static final double kPTurningMotor = 0.1d;
      public static final double kITurningMotor = 0;
      public static final double kDTurningMotor = 0;
      // public static final double kMaxSpeedTurningMotor = 100;
      public static final double kMaxAccelTurningMotor = 1;

      public static final double kRampRateDriveMotor = 0.125d;
      public static final double kPDriveMotor = 0.1d;
      public static final double kIDriveMotor = 0.0001d;
      public static final double kDDriveMotor = 2.5d;
      public static final double kFDriveMotor = 0.048d;

      public static final int kEnterDriveTurningDeadband = 90; //degrees for robot not driving until pod is at target position
      public static final int kExitDriveTurningDeadband = 90; //deg

      public static final double driveSpeedConvertMode = 4000d; 

  }

  public static final class AutoConstants {
      public static final double kMaxSpeedMetersPerSecond = 1d;
      public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond;
      public static final double kMaxAccelerationMetersPerSecondSquared = 0.5d;
      public static final double kMaxAngularAccelerationRadiansPerSecondSquared = 4 * Math.PI;
      public static final double kPXController = 2.5d;
      public static final double kIXController = 0d;
      public static final double kDXController = 0d;
      public static final double kPYController = 2.5d;
      public static final double kIYController = 0d;
      public static final double kDYController = 0d;
      public static final double kPThetaController = 1d;
      public static final double kIThetaController = 0d;
      public static final double kDThetaController = 0d;
  

      public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
          kMaxAngularSpeedRadiansPerSecond, kMaxAngularAccelerationRadiansPerSecondSquared);
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
    public static final int WristMotorID = 10;  // motion profiling
    // Precalculated positions (in motor ticks):
    // {wrist cmd pos, arm cmd pos, inclinator cmd pos, out bucket cmd pos, in bucket cmd pos, initial gripper pos, restractdelay, left gripper open cmd pos, right gripper open cmd pos, left gripper close cmd pos, right gripper close cmd pos}
    public static final double[] intakeBucketPosition = {0, 0, 1, -0.07, 0.07, -0.2, 15, -10, -10, 3};
    // {wrist cmd pos, arm cmd pos, inclinator cmd pos, bucket cmd pos, initial gripper pos, left gripper open cmd pos, right gripper open cmd pos, left gripper close cmd pos, right gripper close cmd pos, smartmotion max velocity, smart motion max accel}
    public static final double[] intakeGroundPosition = {0, 135, 0, 0.05, 75, -12, 0, 3, 3, 5500, 9000};
    // {wrist cmd pos, arm cmd pos, inclinator cmd pos, bucket cmd pos, restractdelay, left gripper open cmd pos, right gripper open cmd pos, left gripper close cmd pos, right gripper close cmd pos, smart motion max vel, smart motion max accel}
    public static final double[] intakeShelfPosition = {0, 85, 0, 0.1, 75, -9, -9, 3, 3, 5500, 9000};
    // {wrist cmd pos, arm cmd pos, inclinator cmd pos, bucket cmd pos, restractdelay, left gripper open cmd pos, right gripper open cmd pos, left gripper close cmd pos, right gripper close cmd pos, smart motion max vel, smart motion max accel}
    public static final double[] scoreHighPosition = {-4, 75, 18.9, 0.1, 70, 3, -12, 3, 3, 5500, 9000};
    // {wrist cmd pos, arm cmd pos, inclinator cmd pos, bucket cmd pos, restractdelay, left gripper open cmd pos, right gripper open cmd pos, left gripper close cmd pos, right gripper close cmd pos, smart motion max vel, smart motion max accel}
    public static final double[] scoreMidPosition = {-4, 80, 2, 0.1, 50, 3, -12, 3, 3, 5500, 9000};
    // {wrist cmd pos, arm cmd pos, inclinator cmd pos, bucket cmd pos, restractdelay, left gripper open cmd pos, right gripper open cmd pos, left gripper close cmd pos, right gripper close cmd pos, smart motion max vel, smart motion max accel}
    public static final double[] scoreGroundPosition = {0, 156, 0, 0.1, 75, 3, 3, -1, -1, 5500, 9000};
    // {wrist cmd pos, arm cmd pos, inclinator cmd pos, bucket cmd pos, left gripper open cmd pos, right gripper open cmd pos, left gripper close cmd pos, right gripper close cmd pos, smart motion max vel, smart motion max accel}
    public static final double[] homePos = {0, 0, 0, 0.07, -12, -12, 3, 3, 2500, 4000};
    // finally constant so saves us the PTA... {Kp, Max, Ki, Min, Kd, Max i, Ki zone, Min i}
    public static final double[] PIDconstants = {0.018, 1, 0, -1, 0.002, 100, 5, -100};

    public static int SmartMotionMaxVel = 2500;
    public static int SmartMotionMaxAccel = 4000;
    public static double BucketInSensorPosition = 0.214286;
    public static double BucketInCommandPosition = 0.07;

    // P Gain, I Gain, D Gain, F Gain, I Zone, Min Output, Max Output
    public static final double[] GripperLGains = {0.3, 0, 0, 0, 0, -1, 1};
    public static final double[] GripperRGains = {0.3, 0, 0, 0, 0, -1, 1};
    public static final double[] WristGains = {0.4, 0, 0, 0, 0, -0.5, 0.5};
    public static final double[] BucketGains = {0.4, 0, 0, 0, 0, -0.25, 0.25};

    public static int gripBoxTicks = 0000;
    public static int gripConeTicks = 0000;
    public static int microAdjustArmTicks = 0000;
    public static int microAdjustElevatorTicks = 0000;
    public static int microAdjustWristTicks = 0000;
  }

  public static final class OIConstants {
      public static final int kDriverControllerPort = 0;
      public static final int kCoDriverControllerPort = 1;

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

      public static final double kDeadbandSteer = 0.1d;
      public static final double kDeadbandDrive = 0.03d;

      public static final double driverMultiplier = 0.75;
      public static final double driverTopMultiplier = 1.5;
      public static final double driverPower = 3.5;//2.5 faster but clicks
      public static final double driverBaseSpeedMetersPerSecond = 00;
      public static final double triggerMultiplier = 0.1;
      public static final double triggerDeadband = 0.1;

      public static final double driverEPower = 3.5;
      public static final double driverEXPMultiplier = driverMultiplier*Math.pow(Math.E, -driverEPower);
      public static final double driverTopEXPMultiplier = driverTopMultiplier*Math.pow(Math.E, -driverEPower);
      public static final double driverEXPJoyMultiplier = driverEPower;

      public static final double targetTurnGainScheduleSpeed = 40;

      public static final double joystickTurningGain = 2;

  }

  public static class VisionConstants {
    // public static final Transform3d robotToCam =
    //         new Transform3d(
    //                 new Translation3d(0.2, 0.0, 0.4),
    //                 new Rotation3d(
    //                         0, 0.524,
    //                         0)); // Cam mounted facing forward, half a meter forward of center, half a meter up
    // from center.
   public static final String aprTagCameraName = "photonvision";
   public static final String coneCameraName = "USB_Web_Camera";

   //For color pipelines
   public static final double CAMERA_HEIGHT_METERS = 0.168;
   public static final double CONE_HEIGHT_METERS = 0.32;
   public static final double CAMERA_PITCH_RADIANS = 0;

   public static final int CUBE_PIPELINE_INDEX = 0;
   public static final int CONE_PIPELINE_INDEX = 1;

   
/**
         * Physical location of the camera on the robot, relative to the center of the robot.
         */
        public static final Transform2d CAMERA_TO_ROBOT = 
            new Transform2d(new Translation2d(0.2, 0.0), new Rotation2d(0.0));

                /** Physical location of the apriltag camera on the robot, relative to the center of the robot. */
        public static final Transform3d APRILTAG_CAMERA_TO_ROBOT =
        new Transform3d(new Translation3d(0.2, 0, -0.90), new Rotation3d(0.0, 0.0, -0.1));

      // Vision Drive Constants

        public static final double TRANSLATION_TOLERANCE = 0.2;
        public static final double ROTATION_TOLERANCE =5;

        public static final double MAX_VELOCITY = 2;
        public static final double MAX_ACCELARATION = 1;
        public static final double MAX_VELOCITY_ROTATION = 8;
        public static final double MAX_ACCELARATION_ROTATION = 8;

        public static final double kPXController = 2.5d;
        public static final double kIXController = 0d;
        public static final double kDXController = 0d;
        public static final double kPYController = 2.5d;
        public static final double kIYController = 0d;
        public static final double kDYController = 0d;
        public static final double kPThetaController = 1d;
        public static final double kIThetaController = 0d;
        public static final double kDThetaController = 0d; 


      }



}
