package frc.robot.subsystems;
import frc.robot.Constants.ArmConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristSubsystem extends SubsystemBase {
    private final CANSparkMax WristMotor;
    private SparkMaxPIDController WristMotorPIDController;
    private RelativeEncoder WristEncoder;

    public WristSubsystem() {
        WristMotor = new CANSparkMax(ArmConstants.GripperMotorLID, CANSparkMax.MotorType.kBrushless);

        WristMotorPIDController = WristMotor.getPIDController();
    }

    public void setPosition(int position) {
        WristMotorPIDController.setReference(position, CANSparkMax.ControlType.kPosition);
    }
}

