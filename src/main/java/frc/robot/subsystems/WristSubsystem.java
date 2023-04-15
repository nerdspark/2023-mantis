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

    private boolean shouldOverridePosition;

    private double overridePosition;

    public void setPositionOverride(boolean shouldOverride, double pos) {
        shouldOverridePosition = shouldOverride;
        overridePosition = pos;
    }

    public boolean isPositionOverridden() {
        return shouldOverridePosition;
    }

    public double getOverridenPosition() {
        return overridePosition;
    }

    public void setPositionOverride(boolean shouldOverride) {
        shouldOverridePosition = shouldOverride;
    }

    public WristSubsystem() {
        wristMotor = new CANSparkMax(ArmConstants.WristMotorID, CANSparkMax.MotorType.kBrushless);
        wristEncoder = wristMotor.getEncoder();
        wristMotorPIDController = wristMotor.getPIDController();

        wristEncoder.setPosition(0);
    }

    public void setPosition(double position) {
        if (shouldOverridePosition) {
            position = overridePosition;
        }
        wristMotorPIDController.setReference(position, CANSparkMax.ControlType.kPosition);
    }

    public double getPosition() {
        return wristEncoder.getPosition();
    }
}
