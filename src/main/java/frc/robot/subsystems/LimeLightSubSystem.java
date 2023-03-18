package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;


public class LimeLightSubSystem extends SubsystemBase{

    private NetworkTable limelightNetworkTable;
    private double currentTimestamp;
    private  double lastAllianceUpdate;
    private Alliance currentAlliance;
    private String limelightHostname;

    private double activePipelineId;
    private boolean enabled = false;
    private boolean driverMode;

    
    public LimeLightSubSystem(String hostname) { 
        limelightHostname = hostname;
        limelightNetworkTable = NetworkTableInstance.getDefault().getTable(limelightHostname);
        currentTimestamp = 0.0;
        lastAllianceUpdate = Double.NEGATIVE_INFINITY;
        currentAlliance = Alliance.Invalid;


    }

    public double getValidTargetUpdateTimestamp(){
        return limelightNetworkTable.getEntry("tv").getLastChange();
    }

    public boolean hasLimelightUpdatedRecently(){
        return getValidTargetUpdateTimestamp()/ 1000000 > currentTimestamp - 1.0;
    }

    public Alliance getCurrentAlliance(){
        return currentAlliance;
    }

    /**
     * Update the limelight pose relative to the center of the drive base on the floor.
     * @param metersForwardOfCenter meters forward or backward of the centerline (where the cross member bar is). Forward is positive.
     * @param metersLeftOrRight meters left or right of the middle of the robot (the centerline between the left and right wheel sets). Right is positive.
     * @param metersUpOrDown meters up or down. Negative means the limelight has clipped into the floor, so don't use negative values for this.
     * @param yaw rotation of the limelight relative to forwards on the robot being 0. In Degrees.
     * @param pitch whether the limelight is angled up or down. In Degrees.
     * @param roll limelight skew. Let's try to mount the limelight so this is always 0. In Degrees.
     */
    public void updateLimelightPose(double metersForwardOfCenter, double metersLeftOrRight, double metersUpOrDown, double yaw, double pitch, double roll){
        LimelightHelpers.setCameraPose_RobotSpace(limelightHostname, metersForwardOfCenter, metersLeftOrRight, metersUpOrDown, yaw, pitch, roll);
    }

    public double getAprilTagId(){
        return LimelightHelpers.getFiducialID(limelightHostname);
    }

    public double getTargetTx(){ //Yaw
        return LimelightHelpers.getTX(limelightHostname);
    }

    public double getTargetTy(){ //Pitch?
        return LimelightHelpers.getTY(limelightHostname);
    }

    public double getTargetTa(){ //Area percentage
        return LimelightHelpers.getTA(limelightHostname);
    }

    /**
     * Get robot pose based on apriltags
     * @return
     */
    public Pose2d getBotPose(){
        switch(currentAlliance){
            case Blue:

                return LimelightHelpers.getBotPose2d_wpiBlue(limelightHostname);
            case Red:
                return LimelightHelpers.getBotPose2d_wpiRed(limelightHostname);
            default:
            case Invalid:
                DriverStation.reportError("VisionSubystem.java: Could not get bot pose. Invalid Alliance.", true);
                return null;
            
        }
    }

    @Override
    public void periodic() {
        outputToSmartDashboard();
        currentTimestamp = Timer.getFPGATimestamp();
        //Check alliance every 5 seconds, hopesfully this will update in disabled. If not it's on the dashboard so we can thumbs-down.
        if(lastAllianceUpdate + 2.5 < currentTimestamp){
            currentAlliance = DriverStation.getAlliance();
            lastAllianceUpdate = currentTimestamp;
        }

       // Flush NetworkTable to send LED mode and pipeline updates immediately
    var shouldFlush = (limelightNetworkTable.getEntry("ledMode").getDouble(0.0) != (enabled ? 0.0 : 1.0) || 
        limelightNetworkTable.getEntry("pipeline").getDouble(0.0) != activePipelineId);
    
        limelightNetworkTable.getEntry("ledMode").setDouble(enabled ? 0.0 : 1.0);
        limelightNetworkTable.getEntry("camMode").setDouble(driverMode ? 1.0 : 0.0);
        limelightNetworkTable.getEntry("pipeline").setDouble(activePipelineId);
  
        if (shouldFlush)  {
        NetworkTableInstance.getDefault().flush();
        }

    }   

    public boolean hasAprilTagTarget(){
        return LimelightHelpers.getFiducialID(limelightHostname) != -1  && LimelightHelpers.getTV(limelightHostname);
    }

    public LimelightResults getLimeLightResults(){
        
        return LimelightHelpers.getLatestResults(limelightHostname);
    }

    public void outputToSmartDashboard() {

        SmartDashboard.putBoolean("Limelight Connection", hasLimelightUpdatedRecently());
        SmartDashboard.putString("Vision Subsystem current Alliance", currentAlliance.name());
    }

    public double getLastTimeStampForPose(){
        return this.currentTimestamp - (LimelightHelpers.getLatency_Pipeline(limelightHostname)/1000) - (LimelightHelpers.getLatency_Capture(limelightHostname)/1000);
    }

    public Pose3d getTargetPose3d_RobotSpace(){
        return LimelightHelpers.getTargetPose3d_RobotSpace(limelightHostname);
    }

    public Pose3d getRobotPose3d_TargetSpace(){
        return LimelightHelpers.getBotPose3d_TargetSpace(limelightHostname);
    }
   
    public void setPipelineId(int pipelineId) {
        activePipelineId = pipelineId;
        LimelightHelpers.setPipelineIndex(limelightHostname, pipelineId);
      }
      
}