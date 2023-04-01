// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class TimeOfFlightSubsystem extends SubsystemBase {
    com.playingwithfusion.TimeOfFlight timeOfFlight;
    //    MedianFilter filter = new MedianFilter(5);

    //    double range;

    /** Creates a new ExampleSubsystem. */
    public TimeOfFlightSubsystem() {
        this.timeOfFlight = new TimeOfFlight(15);
        timeOfFlight.setRangingMode(RangingMode.Short, 36);
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("sensor position", sensor1.getAbsolutePosition());
        //        range = filter.calculate(timeOfFlight.getRange());
        SmartDashboard.putNumber("TOF Range", this.getRange());
        SmartDashboard.putBoolean(
                "cube pickup tof",
                (RobotContainer.getArmSubsystem().getArmPositionState() == ArmSubsystem.ArmPosition.CUBE_PICKUP)
                        && (this.getRange() < 75)
                        && (this.getRange() > 30));
    }

    // public void printSensor() {
    // SmartDashboard.putNumber("sensor position", sensor1.getAbsolutePosition());
    // }
    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public double getRange() {
        return timeOfFlight.getRange();
    }

    public void closeTimeOfFlight() {
        timeOfFlight.close();
    }
}
