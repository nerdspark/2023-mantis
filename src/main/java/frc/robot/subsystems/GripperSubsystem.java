package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class GripperSubsystem extends SubsystemBase {
    private final CANSparkMax leftGripperMotor;
    private final CANSparkMax rightGripperMotor;
    private final SparkMaxPIDController leftGripperMotorPIDController, rightGripperMotorPIDController;
    private final RelativeEncoder leftGripperMotorEncoder, rightGripperMotorEncoder;

    private boolean setSwap = false;

    public GripperSubsystem() {
        leftGripperMotor = new CANSparkMax(ArmConstants.GripperMotorLID, CANSparkMax.MotorType.kBrushless);
        rightGripperMotor = new CANSparkMax(ArmConstants.GripperMotorRID, CANSparkMax.MotorType.kBrushless);

        leftGripperMotorPIDController = leftGripperMotor.getPIDController();
        rightGripperMotorPIDController = rightGripperMotor.getPIDController();

        leftGripperMotorEncoder = leftGripperMotor.getEncoder();
        rightGripperMotorEncoder = rightGripperMotor.getEncoder();

        leftGripperMotorEncoder.setPosition(0);
        rightGripperMotorEncoder.setPosition(0);
    }

    public void setSwap(boolean swap) {
        setSwap = swap;
    }

    public void periodic() {
        SmartDashboard.putNumber("Right Gripper Position", rightGripperMotorEncoder.getPosition());
        SmartDashboard.putNumber("Left Gripper Position", leftGripperMotorEncoder.getPosition());
    }

    public void setLeftPosition(double position) {
        if (setSwap) {
            rightGripperMotorPIDController.setReference(position, CANSparkMax.ControlType.kPosition);
            return;
        }
        leftGripperMotorPIDController.setReference(-position, CANSparkMax.ControlType.kPosition);
    }

    public void setRightPosition(double position) {
        if (setSwap) {
            leftGripperMotorPIDController.setReference(-position, CANSparkMax.ControlType.kPosition);
            return;
        }
        rightGripperMotorPIDController.setReference(position, CANSparkMax.ControlType.kPosition);
    }

    public double getLeftPosition() {
        return leftGripperMotorEncoder.getPosition();
    }

    public double getRightPosition() {
        return rightGripperMotorEncoder.getPosition();
    }
}
