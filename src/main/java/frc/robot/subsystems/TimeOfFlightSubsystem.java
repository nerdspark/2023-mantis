// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TimeOfFlightSubsystem extends SubsystemBase {
    TimeOfFlight timeOfFlight;
    //    MedianFilter filter = new MedianFilter(5);

    public TimeOfFlightSubsystem() {
        this.timeOfFlight = new TimeOfFlight(15);
        timeOfFlight.setRangingMode(RangingMode.Short, 24);
    }

    @Override
    public void periodic() {
        //        SmartDashboard.putNumber("TOF Reading with mean", filter.calculate(timeOfFlight.getRange()));
        //        SmartDashboard.putNumber("TOF Reading without mean", timeOfFlight.getRange());
    }

    public double getRange() {
        return timeOfFlight.getRange();
    }
}
