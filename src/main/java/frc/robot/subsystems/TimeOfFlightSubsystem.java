// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TimeOfFlightSubsystem extends SubsystemBase {
    TimeOfFlight timeOfFlight;

    private double[] lastValues = new double[10];

    public TimeOfFlightSubsystem() {
        this.timeOfFlight = new TimeOfFlight(15);
        timeOfFlight.setRangingMode(RangingMode.Long, 24);
    }

    @Override
    public void periodic() {
        double range = timeOfFlight.getRange();
        for (int i = 0; i < lastValues.length - 1; i++) {
            lastValues[i] = lastValues[i + 1];
        }
        lastValues[lastValues.length - 1] = range;
    }

    public double getRange() {
        return timeOfFlight.getRange();
    }

    public boolean lastValuesWithinBounds(double lowerBound, double upperBound) {
        for (double value : lastValues) {
            if (value < lowerBound || value > upperBound) {
                return false;
            }
        }
        return true;
    }
}
