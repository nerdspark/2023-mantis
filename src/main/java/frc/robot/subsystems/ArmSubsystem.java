// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants.ArmConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.*;



/** Add your docs here. */
public class ArmSubsystem {

    private final CANSparkMax ElevatorMotor1;
    private final CANSparkMax ElevatorMotor2;
    private final CANSparkMax ArmMotor1;
    private final CANSparkMax ArmMotor2;
    
    private final CANSparkMax WristMotor;

    private final CANSparkMax GripperMotor1;
    private final CANSparkMax GripperMotor2;
    private RelativeEncoder EncoderGripperMotor2;
    private RelativeEncoder EncoderGripperMotor1;
    private RelativeEncoder EncoderWristMotor;
    private RelativeEncoder EncoderArmMotor1;
    private RelativeEncoder EncoderArmMotor2;
    private RelativeEncoder EncoderElevatorMotor1;
    private RelativeEncoder EncoderElevatorMotor2;  
    private SparkMaxPIDController ElevatorMotor1PIDController, ElevatorMotor2PIDController, ArmMotor1PIDController, ArmMotor2PIDController, WristMotorPIDController, GripperMotor1PIDController, GripperMotor2PIDController;

    public ArmSubsystem() {
        ElevatorMotor1 = new CANSparkMax(ArmConstants.ElevatorMotor1ID, CANSparkMax.MotorType.kBrushless);
        ElevatorMotor2 = new CANSparkMax(ArmConstants.ElevatorMotor2ID, CANSparkMax.MotorType.kBrushless);
        ElevatorMotor2.setInverted(true);

        ArmMotor1 = new CANSparkMax(ArmConstants.ArmMotor1ID, CANSparkMax.MotorType.kBrushless);
        ArmMotor2 = new CANSparkMax(ArmConstants.ArmMotor2ID, CANSparkMax.MotorType.kBrushless);
        ArmMotor2.setInverted(true);
        
        WristMotor = new CANSparkMax(ArmConstants.WristMotorID, CANSparkMax.MotorType.kBrushless);
        
        GripperMotor1 = new CANSparkMax(ArmConstants.GripperMotor1ID, CANSparkMax.MotorType.kBrushless);
        GripperMotor2 = new CANSparkMax(ArmConstants.GripperMotor2ID, CANSparkMax.MotorType.kBrushless);
        GripperMotor2.setInverted(true);

        EncoderGripperMotor2 = GripperMotor2.getEncoder();
        EncoderGripperMotor1 = GripperMotor1.getEncoder();
        EncoderWristMotor = WristMotor.getEncoder();
        EncoderArmMotor1 = ArmMotor1.getEncoder();
        EncoderArmMotor2 = ArmMotor2.getEncoder();
        EncoderElevatorMotor1 = ElevatorMotor1.getEncoder();
        EncoderElevatorMotor2 = ElevatorMotor2.getEncoder();

        ElevatorMotor1PIDController = ElevatorMotor1.getPIDController();
        ElevatorMotor2PIDController = ElevatorMotor2.getPIDController();
        ArmMotor1PIDController = ArmMotor1.getPIDController();
        ArmMotor2PIDController = ArmMotor2.getPIDController();
        WristMotorPIDController = WristMotor.getPIDController();
        GripperMotor1PIDController = GripperMotor1.getPIDController();
        GripperMotor2PIDController = GripperMotor2.getPIDController();
        
    }

    public void goToPosition(int[] position) {
        ElevatorMotor1PIDController.setReference(position[0], CANSparkMax.ControlType.kPosition);
        ElevatorMotor2PIDController.setReference(position[0], CANSparkMax.ControlType.kPosition);
        ArmMotor1PIDController.setReference(position[1], CANSparkMax.ControlType.kPosition);
        ArmMotor2PIDController.setReference(position[1], CANSparkMax.ControlType.kPosition);
        WristMotorPIDController.setReference(position[2], CANSparkMax.ControlType.kPosition);
    }

    public void gripBox() {
        GripperMotor1PIDController.setReference(ArmConstants.gripBoxTicks, CANSparkMax.ControlType.kPosition);
        GripperMotor2PIDController.setReference(ArmConstants.gripBoxTicks, CANSparkMax.ControlType.kPosition);
    }
    public void gripCone() {
        GripperMotor1PIDController.setReference(ArmConstants.gripConeTicks, CANSparkMax.ControlType.kPosition);
        GripperMotor2PIDController.setReference(ArmConstants.gripConeTicks, CANSparkMax.ControlType.kPosition);
    }
    
    public void releaseGrip() {
        GripperMotor1PIDController.setReference(0, CANSparkMax.ControlType.kPosition);
        GripperMotor2PIDController.setReference(0, CANSparkMax.ControlType.kPosition);
    }

    // multiplier is -1 to 1
    public void microAdjustArm(double multiplier) {
        ArmMotor1PIDController.setReference(EncoderArmMotor1.getPosition() + multiplier * ArmConstants.microAdjustArmTicks, CANSparkMax.ControlType.kPosition);
        ArmMotor2PIDController.setReference(EncoderArmMotor2.getPosition() + multiplier * ArmConstants.microAdjustArmTicks, CANSparkMax.ControlType.kPosition);
    }
    public void microAdjustWrist(double multiplier) {
        WristMotorPIDController.setReference(EncoderWristMotor.getPosition() + multiplier * ArmConstants.microAdjustWristTicks, CANSparkMax.ControlType.kPosition);
    }


}