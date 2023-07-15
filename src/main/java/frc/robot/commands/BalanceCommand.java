package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.BalanceConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * Command to balance the charge station
 */
public class BalanceCommand extends CommandBase {

    private final SwerveSubsystem drivetrainSubsystem;
    private boolean done;
    private double prevRoll = 0;
    private double roll = 0;

    public BalanceCommand(SwerveSubsystem drivetrainSubsystem, boolean isReverse) {
        this.drivetrainSubsystem = drivetrainSubsystem;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        done = false;
        prevRoll = drivetrainSubsystem.getRoll();
    }

    @Override
    public void execute() {
        prevRoll = roll;
        roll = drivetrainSubsystem.getRoll();
        if (prevRoll == 0) {
            prevRoll = drivetrainSubsystem.getRoll();
            return;
        }

        // double speed = 0.35;
        double speed = BalanceConstants.balanceSpeed;

        //        if (done) {
        //            speed = BalanceConstants.slowerBalanceSpeed;
        //        }

        // if ((Math.abs(prevRoll - roll) > 0.025 || done && Math.abs(roll) < 5)) {
        //     if (Math.abs(roll) > 12) return;

        if ((Math.abs(prevRoll - roll) > BalanceConstants.balanceRate
                || done && Math.abs(roll) < BalanceConstants.balanceHold)) {
            if (Math.abs(roll) > BalanceConstants.balanceEntry) return;
            done = true;
            drivetrainSubsystem.setWheelsToX();
            // new WaitCommand(2);

        } else {
            if (roll > 0) {
                speed *= -1;
            }
            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(speed, 0.0, 0.0);
            SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
            drivetrainSubsystem.setModuleStates(moduleStates);
        }
    }
}
