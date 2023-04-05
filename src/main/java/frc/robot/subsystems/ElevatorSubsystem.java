package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ElevatorSubsystem extends SubsystemBase {
    private final CANSparkMax elevatorMotor1;
    private final CANSparkMax elevatorMotor2;
    private final SparkMaxPIDController elevatorMotor1PIDController, elevatorMotor2PIDController;
    private final RelativeEncoder elevatorMotor1Encoder, elevatorMotor2Encoder;

    public ElevatorSubsystem() {
        elevatorMotor1 = new CANSparkMax(ArmConstants.InclinovatorMotor1ID, CANSparkMax.MotorType.kBrushless);
        elevatorMotor2 = new CANSparkMax(ArmConstants.InclinovatorMotor2ID, CANSparkMax.MotorType.kBrushless);

        elevatorMotor1PIDController = elevatorMotor1.getPIDController();
        elevatorMotor2PIDController = elevatorMotor2.getPIDController();

        elevatorMotor1Encoder = elevatorMotor1.getEncoder();
        elevatorMotor2Encoder = elevatorMotor2.getEncoder();

        elevatorMotor1Encoder.setPosition(0);
        elevatorMotor2Encoder.setPosition(0);

        elevatorMotor2.follow(elevatorMotor1);
    }

    public void setPosition(double position) {
        elevatorMotor1PIDController.setReference(-position, CANSparkMax.ControlType.kPosition);
    }

    public double getPosition() {
        return elevatorMotor1Encoder.getPosition();
    }
}
