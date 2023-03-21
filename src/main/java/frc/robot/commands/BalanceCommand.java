package frc.robot.commands;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * Command to balance the charge station
 */
public class BalanceCommand extends CommandBase {

  private final SwerveSubsystem drivetrainSubsystem;
  private final boolean isReverse;
  private boolean done;

  public BalanceCommand(SwerveSubsystem drivetrainSubsystem, boolean isReverse) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.isReverse = isReverse;

    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    done = false;
  }

  @Override
  public void execute() {
    var yAccel = drivetrainSubsystem.getGyroVelocityXYZ()[1];
    if (isReverse) {
      yAccel *= -1;
    }
    if (yAccel > 10 || done) {
      done = true;
      drivetrainSubsystem.setWheelsToX();
    } else {
      var speed = 0.5;
      if (isReverse) {
        speed *= -1;
      }

      
     ChassisSpeeds chassisSpeeds = new ChassisSpeeds(speed, 0.0, 0.0);
      SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
      drivetrainSubsystem.setModuleStates(moduleStates);
    }
  }

//   @Override
//   public void end(boolean interrupted) {
//     drivetrainSubsystem.setWheelsToX();
//   }

}