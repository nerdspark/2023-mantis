package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmSubsystem;


public class ArmJoystickCmd extends CommandBase {

    private final ArmSubsystem armSubsystem;
    private final Supplier<Double> ArmMicroAdjust, WristMicroAdjust;
    private final Supplier<Boolean> isBucketPickup, isGroundPickup, isShelfPickup, isGroundDrop, isMidDrop, isHighDrop;
    private final Supplier<Integer> DPAD;
    private final PIDController targetTurnController = new PIDController(DriveConstants.kPTargetTurning, DriveConstants.kITargetTurning, DriveConstants.kDTargetTurning);

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
        
    }

    @Override
    public void end(boolean interrupted) {
        // swerveSubsystem.stopModules();
    }
    
}
