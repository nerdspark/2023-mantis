package frc.robot.subsystems;

import frc.robot.Constants.ArmConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BucketSubsystem extends SubsystemBase {
    private final CANSparkMax LeftBucketMotor;
    private final CANSparkMax RightBucketMotor;
    private SparkMaxPIDController LeftBucketMotorPIDController, RightBucketMotorPIDController;
    private RelativeEncoder BucketMotor1Encoder, BucketMotor2Encoder;

    public BucketSubsystem() {
        LeftBucketMotor = new CANSparkMax(ArmConstants.BucketMotorLID, CANSparkMax.MotorType.kBrushless);
        RightBucketMotor = new CANSparkMax(ArmConstants.BucketMotorRID, CANSparkMax.MotorType.kBrushless);

        LeftBucketMotorPIDController = LeftBucketMotor.getPIDController();
        RightBucketMotorPIDController = RightBucketMotor.getPIDController();

        BucketMotor1Encoder = LeftBucketMotor.getEncoder();
        BucketMotor2Encoder = RightBucketMotor.getEncoder();
    }

    public double[] getPositions() {
        double[] position = { BucketMotor1Encoder.getPosition(), BucketMotor2Encoder.getPosition() };

        return position;
    }

    // Neither method uses the position parameter
    public void retract() {
        LeftBucketMotorPIDController.setReference(0.84, CANSparkMax.ControlType.kVoltage);
        RightBucketMotorPIDController.setReference(0.84, CANSparkMax.ControlType.kVoltage);

    }

    public void extend() {
        LeftBucketMotorPIDController.setReference(-0.84, CANSparkMax.ControlType.kVoltage);
        RightBucketMotorPIDController.setReference(-0.84, CANSparkMax.ControlType.kVoltage);
    }
}