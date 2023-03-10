package frc.robot.commands;

import java.util.function.Supplier;

import com.fasterxml.jackson.databind.deser.DataFormatReaders.Match;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmSubsystem;


public class ArmJoystickCmd extends CommandBase {

    private final ArmSubsystem armSubsystem;
    private final Supplier<Double> ArmMicroAdjust, WristMicroAdjust;
    private final Supplier<Boolean> isBucketPickup, isGroundPickup, isShelfPickup, isGroundDrop, isMidDrop, isHighDrop;
    private final Supplier<Integer> DPAD;
    private final PIDController PIDControl = new PIDController(ArmConstants.PIDconstants[0], ArmConstants.PIDconstants[2], ArmConstants.PIDconstants[4]);

    public ArmJoystickCmd(ArmSubsystem armSubsystem,
            Supplier<Double> ArmMicroAdjust, Supplier<Double> WristMicroAdjust,
            Supplier<Boolean> isBucketPickup, Supplier<Boolean> isGroundDrop, Supplier<Boolean> isMidDrop, Supplier<Boolean> isHighDrop, Supplier<Integer> DPAD, Supplier<Boolean> isGroundPickup, Supplier<Boolean> isShelfPickup) {
        this.armSubsystem = armSubsystem;
        this.ArmMicroAdjust = ArmMicroAdjust;
        this.WristMicroAdjust = WristMicroAdjust;
        this.isBucketPickup = isBucketPickup;
        this.isGroundDrop = isGroundDrop;
        this.isMidDrop = isMidDrop;
        this.isHighDrop = isHighDrop;
        this.DPAD = DPAD;
        this.isGroundPickup = isGroundPickup;
        this.isShelfPickup = isShelfPickup;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        double[] thisPosition;
        if(isBucketPickup.get()) {
            thisPosition = ArmConstants.intakeBucketPosition;
        } else if(isGroundPickup.get()) {
            thisPosition = ArmConstants.intakeGroundPosition;
        } else if(isShelfPickup.get()) {
            thisPosition = ArmConstants.intakeShelfPosition;
        } else if(isGroundDrop.get()) {
            thisPosition = ArmConstants.scoreGroundPosition;
        } else if(isMidDrop.get()) {
            thisPosition = ArmConstants.scoreMidPosition;
        } else if(isHighDrop.get()) {
            thisPosition = ArmConstants.scoreHighPosition;
        } else if(DPAD.get() > 180) {
            thisPosition = ArmConstants.homePos;
        } 
    }

    @Override
    public void end(boolean interrupted) {
        // swerveSubsystem.stopModules();
    }
    
}
