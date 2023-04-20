// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmPositionCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmPosition;
import frc.robot.subsystems.WristSubsystem;
import java.util.function.Supplier;

public class MicroAdjustCommand extends CommandBase {
    private final WristSubsystem wristSubsystem;
    private final ArmSubsystem armSubsystem;
    private final Supplier<Double> leftJoystickY;
    private final Supplier<Double> rightJoystickY;

    /** Creates a new MicroAdjustCommand. */
    public MicroAdjustCommand(
            ArmSubsystem armSubsystem,
            WristSubsystem wristSubsystem,
            Supplier<Double> leftJoystickY,
            Supplier<Double> rightJoystickY) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.wristSubsystem = wristSubsystem;
        this.armSubsystem = armSubsystem;
        this.leftJoystickY = leftJoystickY;
        this.rightJoystickY = rightJoystickY;
        addRequirements(armSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double currentArmPositionState = 0;
        double currentWristPositionState = 0;

        ArmPosition armPositionState = armSubsystem.getArmPositionState();

        switch (armPositionState) {
            case GROUND_DROP -> {
                currentArmPositionState = ArmConstants.groundPickupPosition.armCmdPos();
                currentWristPositionState = ArmConstants.groundPickupPosition.wristCmdPos();
            }
            case HIGH_DROP -> {
                currentArmPositionState = ArmConstants.highDropPosition.armCmdPos();
                currentWristPositionState = ArmConstants.highDropPosition.wristCmdPos();
            }
            case MID_DROP -> {
                currentArmPositionState = ArmConstants.midDropPosition.armCmdPos();
                currentWristPositionState = ArmConstants.midDropPosition.wristCmdPos();
            }
            default -> {
                return;
            }
        }

        if (Math.abs(rightJoystickY.get()) > 0.05) {
            armSubsystem.setPosition(currentArmPositionState - rightJoystickY.get() * ArmConstants.armAdjustMultiplier);
        } else {
            armSubsystem.setPosition(currentArmPositionState);
        }

        if (Math.abs(leftJoystickY.get()) > 0.05) {
            wristSubsystem.setPosition(
                    -currentWristPositionState + (leftJoystickY.get() * ArmConstants.wristAdjustMultiplier));
        } else {
            wristSubsystem.setPosition(currentWristPositionState);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
