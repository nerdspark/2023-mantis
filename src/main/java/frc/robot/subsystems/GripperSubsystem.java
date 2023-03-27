package frc.robot.subsystems;
import frc.robot.Constants.ArmConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GripperSubsystem extends SubsystemBase {
    private final CANSparkMax leftGripperMotor;
    private final CANSparkMax rightGripperMotor;
    private SparkMaxPIDController leftGripperMotorPIDController, rightGripperMotorPIDController;
    private RelativeEncoder leftGripperMotorEncoder, rightGripperMotorEncoder;

    public GripperSubsystem() {
        leftGripperMotor = new CANSparkMax(ArmConstants.GripperMotorLID, CANSparkMax.MotorType.kBrushless);
        rightGripperMotor = new CANSparkMax(ArmConstants.GripperMotorRID, CANSparkMax.MotorType.kBrushless);

        leftGripperMotorPIDController = leftGripperMotor.getPIDController();
        rightGripperMotorPIDController = rightGripperMotor.getPIDController();

        leftGripperMotorEncoder = leftGripperMotor.getEncoder();
        rightGripperMotorEncoder = rightGripperMotor.getEncoder();
    }

    public void setLeftPosition(double position) {
        leftGripperMotorPIDController.setReference(-position, CANSparkMax.ControlType.kPosition);
    }

    public void setRightPosition(double position) {
        rightGripperMotorPIDController.setReference(position, CANSparkMax.ControlType.kPosition);
    }

    public double getLeftPosition() {
        return leftGripperMotorEncoder.getPosition();
    }

    public double getRightPosition() {
        return rightGripperMotorEncoder.getPosition();
    }

}

