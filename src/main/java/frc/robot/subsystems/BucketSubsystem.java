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
    }

    public void setLeftPosition(double position) {
        LeftBucketMotorPIDController.setReference(position, CANSparkMax.ControlType.kCurrent);
    }

    public void setRightPosition(double position) {
        RightBucketMotorPIDController.setReference(position, CANSparkMax.ControlType.kCurrent);
    }
}

