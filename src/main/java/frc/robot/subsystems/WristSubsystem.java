package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class WristSubsystem extends SubsystemBase {
    private final CANSparkMax wristMotor;
    private SparkMaxPIDController wristMotorPIDController;
    private RelativeEncoder wristEncoder;

    public WristSubsystem() {
        wristMotor = new CANSparkMax(ArmConstants.WristMotorID, CANSparkMax.MotorType.kBrushless);
        wristEncoder = wristMotor.getEncoder();
        wristMotorPIDController = wristMotor.getPIDController();
    }

    public void setPosition(double position) {
        wristMotorPIDController.setReference(position, CANSparkMax.ControlType.kPosition);
    }

    public double getPosition() {
        return wristEncoder.getPosition();
    }
}
