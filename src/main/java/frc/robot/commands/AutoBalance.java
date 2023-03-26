// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import frc.robot.RobotContainer;
// import frc.robot.Constants.AutoConstants;
// import frc.robot.subsystems.SwerveSubsystem;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.CommandBase;


// /** An example command that uses an example subsystem. */
// public class AutoBalance extends CommandBase {
//   @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

//   // double BalanceKP;
//   // double BalanceKI;
//   // double BalanceKD;

//   private final Timer timer = new Timer();
//   public AutoBalance(SwerveSubsystem swerveSubsystem, double maxVel, double maxAccel) {
//     // BalanceKP = Preferences.getDouble(DriveConstants.kBalancePKey, -0.015);
//     // BalanceKI = Preferences.getDouble(DriveConstants.kBalanceIKey, 0.0);
//     // BalanceKD = Preferences.getDouble(DriveConstants.kBalanceDKey, 0.01);
//     addRequirements(RobotContainer.getSwerveSubsystem());

//     // PIDController balanceController = new PIDController(AutoConstants.kPBalanceController, AutoConstants.kIBalanceController,
//     //  AutoConstants.kDBalanceController);

// }


//   // /**
//   //  * Creates a new ExampleCommand.
//   //  *
//   //  * @param subsystem The subsystem used by this command.
//   //  */
//   // public DriveFollowPath(ExampleSubsystem subsystem) {
//   //   m_subsystem = subsystem;
//   //   // Use addRequirements() here to declare subsystem dependencies.
//   //   addRequirements(subsystem);
//   // }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     RobotContainer.getSwerveSubsystem().enableBrakeMode(true);
//     timer.reset();
//     timer.start();


//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     // double time = timer.get();
//     // BalanceKP = Preferences.getDouble(DriveConstants.kBalancePKey, -0.015);
//     // BalanceKI = Preferences.getDouble(DriveConstants.kBalanceIKey, 0.0);
//     // BalanceKD = Preferences.getDouble(DriveConstants.kBalanceDKey, 0.01);

//     // double BalanceDeadbanDeg = Preferences.getDouble(DriveConstants.autoBalanceDeadbandDegKey, 6);

    // try (PIDController balanceController = new PIDController(AutoConstants.kPBalanceController, AutoConstants.kIBalanceController,
    //   AutoConstants.kDBalanceController)) {
    //   double Roll_Deg = RobotContainer.swerveSubsystem.IMU.getRoll();
    //   if (Math.abs(Roll_Deg) > AutoConstants.BalanceDeadBandDeg) {
    //       double output = balanceController.calculate(Roll_Deg, 0);
    //       // double balanceSpeed = new ChassisSpeeds(output, 0, 0);
    //       RobotContainer.swerveSubsystem.drive(new ChassisSpeeds(output, 0.0, 0.0));
    //   }
    //   else {
    //       RobotContainer.swerveSubsystem.setWheelsToX();
//     try (PIDController balanceController = new PIDController(AutoConstants.kPBalanceController, AutoConstants.kIBalanceController,
//       AutoConstants.kDBalanceController)) {
//       double Roll_Deg = RobotContainer.swerveSubsystem.IMU.getRoll();
//       if (Math.abs(Roll_Deg) > AutoConstants.BalanceDeadBandDeg) {
//           double output = balanceController.calculate(Roll_Deg, 0);
//           // double balanceSpeed = new ChassisSpeeds(output, 0, 0);
//           RobotContainer.swerveSubsystem.drive(new ChassisSpeeds(output, 0.0, 0.0));
//       }
//       else {
//           RobotContainer.swerveSubsystem.setWheelsToX();

    //   }
    // }
  // }
//       }
//     }
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     timer.stop();
//     RobotContainer.getSwerveSubsystem().stopModules();
//     RobotContainer.getSwerveSubsystem().setWheelsToX();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return true;
//     // return timer.hasElapsed(BalanceKD);
//   }
// }
