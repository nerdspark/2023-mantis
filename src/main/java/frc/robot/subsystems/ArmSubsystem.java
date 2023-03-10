package frc.robot.subsystems;
import frc.robot.Constants.ArmConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    private final CANSparkMax ArmMotor;
    private final CANSparkMax ArmMotor2;
    private SparkMaxPIDController ArmMotorPIDController, ArmMotor2PIDController;
    private RelativeEncoder ArmMotorEncoder, ArmMotor2Encoder;

    public ArmSubsystem() {
        ArmMotor = new CANSparkMax(ArmConstants.ArmMotor1ID, CANSparkMax.MotorType.kBrushless);
        ArmMotor2 = new CANSparkMax(ArmConstants.ArmMotor2ID, CANSparkMax.MotorType.kBrushless);

        ArmMotorPIDController = ArmMotor.getPIDController();
        ArmMotor2PIDController = ArmMotor2.getPIDController();

        ArmMotor2.follow(ArmMotor, true);
    }

    public void changeArmSmartMotionParameters(int maxVel, int maxAccel) {
        ArmMotorPIDController.setSmartMotionMaxVelocity(maxVel, 0);
        ArmMotorPIDController.setSmartMotionMaxAccel(maxAccel, 0);
    }

    public void goToPosition(int position) {
        ArmMotorPIDController.setReference(position, CANSparkMax.ControlType.kSmartMotion);
    }
}

