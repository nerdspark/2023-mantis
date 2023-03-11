package frc.robot.commands;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.BucketSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GripperSubsystem;

public class ArmJoystickCmd extends CommandBase {

    private final ArmSubsystem armSubsystem;
    private final GripperSubsystem gripperSubsystem;
    private final BucketSubsystem bucketSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final Supplier<Double> ArmMicroAdjust, WristMicroAdjust;
    private final Supplier<Boolean> isBucketPickup, isGroundPickup, isShelfPickup, isGroundDrop, isMidDrop, isHighDrop;
    private final Supplier<Integer> DPAD;
    private final PIDController PIDControl = new PIDController(ArmConstants.PIDconstants[0],
            ArmConstants.PIDconstants[2], ArmConstants.PIDconstants[4]);

    public ArmJoystickCmd(ArmSubsystem armSubsystem, GripperSubsystem gripperSubsystem, BucketSubsystem bucketSubsystem,
            ElevatorSubsystem elevatorSubsystem,
            Supplier<Double> ArmMicroAdjust, Supplier<Double> WristMicroAdjust,
            Supplier<Boolean> isBucketPickup, Supplier<Boolean> isGroundDrop, Supplier<Boolean> isMidDrop,
            Supplier<Boolean> isHighDrop, Supplier<Integer> DPAD, Supplier<Boolean> isGroundPickup,
            Supplier<Boolean> isShelfPickup) {
        this.armSubsystem = armSubsystem;
        this.gripperSubsystem = gripperSubsystem;
        this.bucketSubsystem = bucketSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.ArmMicroAdjust = ArmMicroAdjust;
        this.WristMicroAdjust = WristMicroAdjust;
        this.isBucketPickup = isBucketPickup;
        this.isGroundDrop = isGroundDrop;
        this.isMidDrop = isMidDrop;
        this.isHighDrop = isHighDrop;
        this.DPAD = DPAD;
        this.isGroundPickup = isGroundPickup;
        this.isShelfPickup = isShelfPickup;
        addRequirements(armSubsystem, gripperSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        Map<String, Double> thisPosition;
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
        }

        gripperSubsystem.setLeftPosition(thisPosition.get("leftGripperCloseCmdPos"));
        gripperSubsystem.setRightPosition(thisPosition.get("rightGripperCloseCmdPos"));
    }

    @Override
    public void end(boolean interrupted) {
        // swerveSubsystem.stopModules();
    }

}
