package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class ArmJoystickCmd extends CommandBase {
    private final SwerveSubsystem armSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningTargX, turningTargY;
    private final Supplier<Boolean> fieldOrientedFunction, resetGyroButton, cancelTurn, topSpeed;
    private final Supplier<Integer> DPAD;
    private final Supplier<Double> leftTrigger;
    private final Supplier<Double> rightTrigger;
    private final SlewRateLimiter speedLimiter, turningLimiter;
    private final PIDController targetTurnController = new PIDController(DriveConstants.kPTargetTurning, DriveConstants.kITargetTurning, DriveConstants.kDTargetTurning);

    public ArmJoystickCmd() {
            
    }


    private double targetAngle;

    public SwerveJoystickCmd(ArmSubsystem armSubsystem, Joystick joystick) {
        xSpdFunction = () -> joystick.getRawAxis(0);
        ySpdFunction = () -> joystick.getRawAxis(1);
        turningTargX = () -> joystick.getRawAxis(4);
        turningTargY = () -> joystick.getRawAxis(5);
        fieldOrientedFunction = () -> joystick.getRawButton(1);
        resetGyroButton = () -> joystick.getRawButton(2);
        cancelTurn = () -> joystick.getRawButton(3);
        topSpeed = () -> joystick.getRawButton(4);
        DPAD = () -> joystick.getPOV();
        leftTrigger = () -> joystick.getRawAxis(2);
        rightTrigger = () -> joystick.getRawAxis(3);
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);

        speedLimiter = new SlewRateLimiter(DriveConstants.kMaxSpeedChange);
        turningLimiter = new SlewRateLimiter(DriveConstants.kMaxTurningChange);
    };