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
    }

    public void setPosition(double position) {
        ElevatorMotorPIDController.setReference(position, CANSparkMax.ControlType.kPosition);
        ElevatorMotor2PIDController.setReference(position, CANSparkMax.ControlType.kPosition);
    }

    public double[] getPositions() {
        double[] position = {ElevatorMotorEncoder.getPosition(), ElevatorMotor2Encoder.getPosition()};

        return position;
    }
}

