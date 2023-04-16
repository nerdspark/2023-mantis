// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class LimelightSubsystem extends SubsystemBase {

    public NetworkTable defaultTable = NetworkTableInstance.getDefault().getTable(VisionConstants.LIMELIGHT_NAME);
    public NetworkTableEntry tx = defaultTable.getEntry("tx");
    public NetworkTableEntry ty = defaultTable.getEntry("ty");
    public NetworkTableEntry tv = defaultTable.getEntry("tv");
    public NetworkTableEntry tp = defaultTable.getEntry("pipeline");
    public NetworkTableEntry ledMode = defaultTable.getEntry("ledMode");

    public double p;
    public double x;
    public double y;
    public double v;
    public double led;

    /** Creates a new Limelight. */
    public LimelightSubsystem() {

        setPipeline(0);
    }

    @Override
    public void periodic() {

        p = tp.getDouble(-1.0);
        x = tx.getDouble(0.0);
        y = ty.getDouble(0.0);
        v = tv.getDouble(-1.0);
        led = ledMode.getDouble(-1.0);
        SmartDashboard.putNumber("tx", tx.getDouble(0.0));
        // This method will be called once per scheduler run
    }

    public void setPipeline(int pipelineId) {
        defaultTable.getEntry("pipeline").setInteger(pipelineId);
        p = pipelineId;
    }

    public double getPipeline() {
        p = tp.getDouble(-1.0);
        return p;
    }

    public double getX() {
        x = tx.getDouble(0.0);
        return x;
    }

    public double getY() {
        y = ty.getDouble(0.0);
        return y;
    }

    public double getV() {
        v = tv.getDouble(0.0);
        return v;
    }

    public void setLedMode(int mode) {
        this.ledMode.setNumber(mode);
        // this.c_ledMode.setNumber(mode);
    }

    public double getDistanceToTarget(int target) {
        double goalHeightInches = VisionConstants.HIGH_CONE_TARGET_HEIGHT;

        double angleToGoalDegrees = VisionConstants.LIMELIGHT_PITCH + getY();

        double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);

        return (goalHeightInches - VisionConstants.LIMELIGHT_METERS_UP) / Math.tan(angleToGoalRadians);
    }
}
