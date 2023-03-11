package frc.robot.commands;

import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.BucketSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ArmJoystickCmd extends CommandBase {

    private final ArmSubsystem armSubsystem;
    private final GripperSubsystem gripperSubsystem;
    private final BucketSubsystem bucketSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final WristSubsystem wristSubsystem;
    private final Supplier<Double> ArmMicroAdjust, WristMicroAdjust;
    private final Supplier<Boolean> isBucketPickup, isGroundPickup, isShelfPickup, isGroundDrop, isMidDrop, isHighDrop;
    private final Supplier<Integer> DPAD;
    private final PIDController PIDControl = new PIDController(ArmConstants.pidConstants.get("kp"), ArmConstants.pidConstants.get("ki"), ArmConstants.pidConstants.get("kd"));

    public ArmJoystickCmd(ArmSubsystem armSubsystem, GripperSubsystem gripperSubsystem, BucketSubsystem bucketSubsystem,
            ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem, 
            Supplier<Double> ArmMicroAdjust, Supplier<Double> WristMicroAdjust,
            Supplier<Boolean> isBucketPickup, Supplier<Boolean> isGroundDrop, Supplier<Boolean> isMidDrop,
            Supplier<Boolean> isHighDrop, Supplier<Integer> DPAD, Supplier<Boolean> isGroundPickup,
            Supplier<Boolean> isShelfPickup) {
        this.armSubsystem = armSubsystem;
        this.gripperSubsystem = gripperSubsystem;
        this.bucketSubsystem = bucketSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.wristSubsystem = wristSubsystem;

        this.ArmMicroAdjust = ArmMicroAdjust;
        this.WristMicroAdjust = WristMicroAdjust;

        this.isBucketPickup = isBucketPickup;
        this.isGroundDrop = isGroundDrop;
        this.isMidDrop = isMidDrop;
        this.isHighDrop = isHighDrop;
        this.isGroundPickup = isGroundPickup;
        this.isShelfPickup = isShelfPickup;

        this.DPAD = DPAD;

        addRequirements(this.armSubsystem, this.gripperSubsystem, this.bucketSubsystem, this.elevatorSubsystem, this.wristSubsystem);
    }

    @Override
    public void initialize() {
        // Is it safe to initialize to home position?
    }

    @Override
    public void execute() {
        Map<String, Double> thisPosition = null;
        if (isBucketPickup.get()) {
            thisPosition = ArmConstants.intakeBucketPosition;
        } else if (isGroundPickup.get()) {
            thisPosition = ArmConstants.intakeGroundPosition;
        } else if (isShelfPickup.get()) {
            thisPosition = ArmConstants.intakeShelfPosition;
        } else if (isGroundDrop.get()) {
            thisPosition = ArmConstants.scoreGroundPosition;
        } else if (isMidDrop.get()) {
            thisPosition = ArmConstants.scoreMidPosition;
        } else if (isHighDrop.get()) {
            thisPosition = ArmConstants.scoreHighPosition;
        } else if (DPAD.get() > 180) {
            thisPosition = ArmConstants.homePosition;
        } else if (WristMicroAdjust.get() != 0) {
            // If this is correct, need to copy for arm micro adjustment
            wristSubsystem.microAdjust(WristMicroAdjust.get());
        } else { // No buttons pressed
            return;
        }
        
        armSubsystem.goToPosition(thisPosition.get("armCmdPos"));
        gripperSubsystem.setLeftPosition(thisPosition.get("leftGripperCloseCmdPos"));
        gripperSubsystem.setRightPosition(thisPosition.get("rightGripperCloseCmdPos"));

        // TODO: confirm the logic for extending/retracting the bucket
        if (thisPosition.get("bucketCmdPos") >= 0) {
            bucketSubsystem.retract(thisPosition.get("bucketCmdPos"));
        } else {
            bucketSubsystem.extend(thisPosition.get("bucketCmdPos"));
        }

        elevatorSubsystem.setPosition(thisPosition.get("inclinatorCmdPos"));
        wristSubsystem.setPosition(thisPosition.get("wristCmdPos"));


    }

    @Override
    public void end(boolean interrupted) {
        // swerveSubsystem.stopModules();
    }

}
