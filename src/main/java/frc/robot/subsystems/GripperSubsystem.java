package frc.robot.subsystems;
import frc.robot.Constants.ArmConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GripperSubsystem extends SubsystemBase {
    private final CANSparkMax LeftGripperMotor;
    private final CANSparkMax RightGripperMotor;
    private SparkMaxPIDController LeftGripperMotorPIDController, RightGripperMotorPIDController;
    private RelativeEncoder GripperMotor1Encoder, GripperMotor2Encoder;

    public GripperSubsystem() {
        LeftGripperMotor = new CANSparkMax(ArmConstants.GripperMotorLID, CANSparkMax.MotorType.kBrushless);
        RightGripperMotor = new CANSparkMax(ArmConstants.GripperMotorRID, CANSparkMax.MotorType.kBrushless);

        LeftGripperMotor.setInverted(true);

        LeftGripperMotorPIDController = LeftGripperMotor.getPIDController();
        RightGripperMotorPIDController = RightGripperMotor.getPIDController();
    }

    public void setLeftPosition(double position) {
        LeftGripperMotorPIDController.setReference(position, CANSparkMax.ControlType.kPosition);
    }

    public void setRightPosition(double position) {
        RightGripperMotorPIDController.setReference(position, CANSparkMax.ControlType.kPosition);
    }

}

