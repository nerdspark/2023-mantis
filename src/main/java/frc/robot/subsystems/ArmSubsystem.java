package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
    private final CANSparkMax armMotor1;
    private final CANSparkMax armMotor2;
    private final SparkMaxPIDController armMotor1PIDController, armMotor2PIDController;
    private final RelativeEncoder armMotor1Encoder, armMotor2Encoder;

    public enum ArmPosition {
        HOME,
        GROUND_PICKUP,
        BUCKET_PICKUP,
        CUBE_PICKUP,
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
        return switch (armPosition) {
            default -> ArmConstants.homePosition;
            case GROUND_PICKUP -> ArmConstants.groundPickupPosition;
            case BUCKET_PICKUP -> ArmConstants.bucketPickupPosition;
            case SHELF_PICKUP -> ArmConstants.shelfPickupPosition;
            case GROUND_DROP -> ArmConstants.groundDropPosition;
            case MID_DROP -> ArmConstants.midDropPosition;
            case HIGH_DROP -> ArmConstants.highDropPosition;
        };
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
    }

    public void changeArmSmartMotionParameters(double maxVel, double maxAccel) {
        armMotor1PIDController.setSmartMotionMaxVelocity(maxVel, 0);
        armMotor1PIDController.setSmartMotionMaxAccel(maxAccel, 0);
        armMotor2PIDController.setSmartMotionMaxVelocity(maxVel, 0);
        armMotor2PIDController.setSmartMotionMaxAccel(maxAccel, 0);
    }

    public void setPosition(double position) {
        armMotor1PIDController.setReference(-position, CANSparkMax.ControlType.kSmartMotion);
        armMotor2PIDController.setReference(-position, CANSparkMax.ControlType.kSmartMotion);
    }

    public double[] getPositions() {
        return new double[] {armMotor1Encoder.getPosition(), armMotor2Encoder.getPosition()};
    }

    public void setZero() {
        armMotor1.set(0);
        armMotor2.set(0);
        armMotor1Encoder.setPosition(0);
        armMotor2Encoder.setPosition(0);
    }
}
