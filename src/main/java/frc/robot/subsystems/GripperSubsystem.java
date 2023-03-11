package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.GripperConstants;


public class GripperSubsystem extends SubsystemBase {

    private final CANSparkMax leftGripper;
    private final CANSparkMax rightGripper;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    private final PIDController leftGripperPID;
    private final PIDController rightGripperPID;

    public GripperSubsystem(int leftGripperID, int rightGripperID, boolean leftGripperReversed, boolean rightGripperReversed){

        leftGripper = new CANSparkMax(GripperConstants.leftGripperID, MotorType.kBrushless);
        rightGripper = new CANSparkMax(GripperConstants.rightGripperID, MotorType.kBrushless);

        leftGripper.setInverted(GripperConstants.leftGripperReversed);
        rightGripper.setInverted(GripperConstants.rightGripperReversed);

        leftEncoder = leftGripper.getEncoder();
        rightEncoder = rightGripper.getEncoder();

        leftGripperPID = new PIDController(GripperConstants.kPGripper, GripperConstants.kIGripper, GripperConstants.kDGripper);
        rightGripperPID = new PIDController(GripperConstants.kPGripper, GripperConstants.kIGripper, GripperConstants.kDGripper);

        resetEncoders();
    }

    public void resetEncoders(){
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    public double getLGripperPosition () {
        return leftEncoder.getPosition();
    }

    public double getRGripperPosition () {
        return rightEncoder.getPosition();
    }

    public double getLGripperVelocity() {
        return leftEncoder.getVelocity();
    }

    public double getRGripperVelocity() {
        return rightEncoder.getVelocity();
    }


    public void setGripperPosition(int gripperLTarget, int gripperRTarget){
        leftGripper.set(leftGripperPID.calculate(getLGripperPosition(), gripperLTarget));
        rightGripper.set(rightGripperPID.calculate(getRGripperPosition(), gripperRTarget));
    }
    
    public void stopGripper(){
        leftGripper.set(0);
        rightGripper.set(0);
    }
    
}

    // public double getAbsoluteEncoderRad() {
    //     double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
    //     angle *= 2.0 * Math.PI;
    //     angle -= absoluteEncoderOffsetRad;
    //     return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    // }