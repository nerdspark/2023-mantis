package frc.robot.subsystems;

import frc.robot.Constants.ArmConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    private final CANSparkMax armMotor1;
    private final CANSparkMax armMotor2;
    private SparkMaxPIDController armMotor1PIDController, armMotor2PIDController;
    private RelativeEncoder armMotor1Encoder, armMotor2Encoder;

    public enum ArmPosition {
        HOME,
        GROUND_PICKUP,
        BUCKET_PICKUP,
        SHELF_PICKUP,
        GROUND_DROP,
        MID_DROP,
        HIGH_DROP
    }

    public ArmPosition armPosition = ArmPosition.HOME;

    public void setArmPositionState(ArmPosition state) {
        this.armPosition = state;
    }

    public ArmPosition getArmPositionState() {
        return armPosition;
    }

    public ArmConstants.ArmPositionData getCurrentArmPositionData() {
        switch (armPosition) {
            default:
                return ArmConstants.homePosition;
            case GROUND_PICKUP:
                return ArmConstants.groundPickupPosition;
            case BUCKET_PICKUP:
                return ArmConstants.bucketPickupPosition;
            case SHELF_PICKUP:
                return ArmConstants.shelfPickupPosition;
            case GROUND_DROP:
                return ArmConstants.groundDropPosition;
            case MID_DROP:
                return ArmConstants.midDropPosition;
            case HIGH_DROP:
                return ArmConstants.highDropPosition;
        }
    }

    public ArmSubsystem() {
        armMotor1 = new CANSparkMax(ArmConstants.ArmMotorRID, CANSparkMax.MotorType.kBrushless);
        armMotor2 = new CANSparkMax(ArmConstants.ArmMotorLID, CANSparkMax.MotorType.kBrushless);

        armMotor1PIDController = armMotor1.getPIDController();
        armMotor2PIDController = armMotor2.getPIDController();

        armMotor1Encoder = armMotor1.getEncoder();
        armMotor2Encoder = armMotor1.getEncoder();

        armMotor1Encoder.setPosition(0);
        armMotor2Encoder.setPosition(0);

        System.out.println("[ArmSubsystem] " + " Encoder 1 position: " + armMotor1Encoder.getPosition()
                + " Encoder 2 position: " + armMotor2Encoder.getPosition());
    }

    public void changeArmSmartMotionParameters(double maxVel, double maxAccel) {
        armMotor1PIDController.setSmartMotionMaxVelocity(maxVel, 0);
        armMotor1PIDController.setSmartMotionMaxAccel(maxAccel, 0);
        armMotor2PIDController.setSmartMotionMaxVelocity(maxVel, 0);
        armMotor2PIDController.setSmartMotionMaxAccel(maxAccel, 0);
    }

    public void goToPosition(double position) {
        armMotor1PIDController.setReference(-position, CANSparkMax.ControlType.kSmartMotion);
        armMotor2PIDController.setReference(-position, CANSparkMax.ControlType.kSmartMotion);
    }

    public double[] getPositions() {
        double[] position = { armMotor1Encoder.getPosition(), armMotor2Encoder.getPosition() };

        return position;
    }
}