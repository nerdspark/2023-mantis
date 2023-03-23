package frc.robot.subsystems;
import frc.robot.Constants.ArmConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

    public void microAdjust(double position_delta) {
        // Does it make sense to use the encoder like this, instead of the PID controller?
        double currentPosition = wristEncoder.getPosition();
        double newPosition = currentPosition + position_delta;
        wristEncoder.setPosition(newPosition);
    }

    public double getPosition() {
        return wristEncoder.getPosition();
    }
}

