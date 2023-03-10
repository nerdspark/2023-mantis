package frc.robot.subsystems;
import frc.robot.Constants.ArmConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    private final CANSparkMax ArmMotor1;
    private final CANSparkMax ArmMotor2;
    private SparkMaxPIDController ArmMotor1PIDController, ArmMotor2PIDController;
    private RelativeEncoder ArmMotor1Encoder, ArmMotor2Encoder;

    public ArmSubsystem() {
        ArmMotor1 = new CANSparkMax(ArmConstants.ArmMotor1ID, CANSparkMax.MotorType.kBrushless);
        ArmMotor2 = new CANSparkMax(ArmConstants.ArmMotor2ID, CANSparkMax.MotorType.kBrushless);
        ArmMotor2.setInverted(true);

        ArmMotor1PIDController = ArmMotor1.getPIDController();
        ArmMotor2PIDController = ArmMotor2.getPIDController();

        ArmMotor1Encoder = ArmMotor1.getEncoder();
        ArmMotor2Encoder = ArmMotor2.getEncoder();

        ArmMotor2PIDController.setSmartMotionMaxVelocity(ArmConstants.SmartMotionMaxVel, 0);        
        ArmMotor1PIDController.setSmartMotionMaxAccel(ArmConstants.SmartMotionMaxAccel, 0);
    }

    public void goToPosition(int position) {
        ArmMotor1PIDController.setReference(position, CANSparkMax.ControlType.kSmartMotion);
        ArmMotor2PIDController.setReference(position, CANSparkMax.ControlType.kSmartMotion);
    }
}