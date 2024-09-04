package frc.robot.subsystems.Limelight.LimelightIO;

import edu.wpi.first.networktables.NetworkTable;

public class LimelightIOLimelight3 implements LimelightIO{
    NetworkTable networkTable;

    public LimelightIOLimelight3(NetworkTable networkTable) {
        this.networkTable = networkTable;
    }
    
    public boolean isValidTarget() {
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

    public double getDistance() { 
       return Math.sqrt(getAprilTagXCameraSpace() * getAprilTagXCameraSpace() + getAprilTagZCameraSpace() * getAprilTagZCameraSpace());
    }

    public double getAngleOffset() {
        return Math.atan2(getAprilTagZCameraSpace(), getAprilTagXCameraSpace());
    } 

    public void updateInputs(LimelightIOInputs inputs) {
       inputs.aprilTagID = (int)networkTable.getEntry("tid").getInteger(0);
       inputs.validTargetExists = isValidTarget();
       inputs.aprilTagXCameraSpace = getAprilTagXCameraSpace();
       inputs.aprilTagZCameraSpace = getAprilTagZCameraSpace();
    }
}
