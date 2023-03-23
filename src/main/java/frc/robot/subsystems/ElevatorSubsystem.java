package frc.robot.subsystems;

import frc.robot.Constants.ArmConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private final CANSparkMax ElevatorMotor;
    private final CANSparkMax ElevatorMotor2;
    private SparkMaxPIDController ElevatorMotorPIDController, ElevatorMotor2PIDController;
    private RelativeEncoder ElevatorMotorEncoder, ElevatorMotor2Encoder;

    public ElevatorSubsystem() {
        ElevatorMotor = new CANSparkMax(ArmConstants.InclinovatorMotor1ID, CANSparkMax.MotorType.kBrushless);
        ElevatorMotor2 = new CANSparkMax(ArmConstants.InclinovatorMotor2ID, CANSparkMax.MotorType.kBrushless);

        ElevatorMotorPIDController = ElevatorMotor.getPIDController();
        ElevatorMotor2PIDController = ElevatorMotor2.getPIDController();

        ElevatorMotorEncoder = ElevatorMotor.getEncoder();
        ElevatorMotor2Encoder = ElevatorMotor2.getEncoder();

        ElevatorMotorEncoder.setPosition(0);
        ElevatorMotor2Encoder.setPosition(0);

        ElevatorMotor2.follow(ElevatorMotor);
    }

    public void setPosition(double position) {
        System.out.println("ElevatorSubsystem: " + position);
        ElevatorMotorPIDController.setReference(-position, CANSparkMax.ControlType.kPosition);
    }

    public double getPosition() {
        return ElevatorMotorEncoder.getPosition();
    }
}
