package frc.robot.subsystems.Limelight.LimelightIO;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import frc.robot.LimelightHelpers;

public class LimelightIOLimelight3 implements LimelightIO{
    private NetworkTable networkTable;
    private Pose2d latestVisionPose;

    public LimelightIOLimelight3(NetworkTable networkTable) {
        this.networkTable = networkTable;
    }
    
    public boolean getValidTargetExists() {
        switch ((int)networkTable.getEntry("tv").getInteger(0)) {
            case 0:
                return false;
            case 1:
                return true;
        }
        return false;
    }

    private double[] getAprilTagPoseCameraSpace() {
        return networkTable.getEntry("targetpose_cameraspace").getDoubleArray(new double[]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    }

    private double getAprilTagXCameraSpace() {
        return getAprilTagPoseCameraSpace()[0];
    }

    private double getAprilTagZCameraSpace() {
        return getAprilTagPoseCameraSpace()[2];
    }

    public double getDistance() { // returns in cm
       return Math.sqrt(getAprilTagXCameraSpace() * getAprilTagXCameraSpace() + getAprilTagZCameraSpace() * getAprilTagZCameraSpace()) * 100;
    }

    public double getAngleOffset() {
        return Math.atan2(getAprilTagXCameraSpace(), getAprilTagZCameraSpace());
    } 

    public double getTx() {
        return networkTable.getEntry("tx").getDouble(0);
    }

    public int getId() {
        return (int)networkTable.getEntry("tid").getInteger(0);
    }

    public void updateInputs(LimelightIOInputs inputs) {
       inputs.aprilTagID = (int)networkTable.getEntry("tid").getInteger(0);
       inputs.validTargetExists = getValidTargetExists();
       inputs.aprilTagXCameraSpace = getAprilTagXCameraSpace();
       inputs.aprilTagZCameraSpace = getAprilTagZCameraSpace();
       inputs.distance = getDistance();
       inputs.angleOffset = getAngleOffset() * (180/Math.PI);
       inputs.tx = getTx();
    }
}
