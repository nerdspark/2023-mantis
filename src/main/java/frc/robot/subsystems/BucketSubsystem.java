package frc.robot.subsystems;

import frc.robot.Constants.ArmConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BucketSubsystem extends SubsystemBase {
    private final CANSparkMax leftBucketMotor;
    private final CANSparkMax rightBucketMotor;
    private SparkMaxPIDController leftBucketMotorPIDController, rightBucketMotorPIDController;
    private RelativeEncoder leftBucketMotorEncoder, rightBucketMotorEncoder;

    public BucketSubsystem() {
        leftBucketMotor = new CANSparkMax(ArmConstants.BucketMotorLID, CANSparkMax.MotorType.kBrushless);
        rightBucketMotor = new CANSparkMax(ArmConstants.BucketMotorRID, CANSparkMax.MotorType.kBrushless);

        leftBucketMotorPIDController = leftBucketMotor.getPIDController();
        rightBucketMotorPIDController = rightBucketMotor.getPIDController();

        leftBucketMotorEncoder = leftBucketMotor.getEncoder();
        rightBucketMotorEncoder = rightBucketMotor.getEncoder();
    }

    public double[] getPositions() {
        double[] position = { leftBucketMotorEncoder.getPosition(), rightBucketMotorEncoder.getPosition() };

        return position;
    }

    // Neither method uses the position parameter
    public void retract() {
        leftBucketMotorPIDController.setReference(0.84, CANSparkMax.ControlType.kVoltage);
        rightBucketMotorPIDController.setReference(0.84, CANSparkMax.ControlType.kVoltage);

    }

    public void extend() {
        leftBucketMotorPIDController.setReference(-0.84, CANSparkMax.ControlType.kVoltage);
        rightBucketMotorPIDController.setReference(-0.84, CANSparkMax.ControlType.kVoltage);
    }
}