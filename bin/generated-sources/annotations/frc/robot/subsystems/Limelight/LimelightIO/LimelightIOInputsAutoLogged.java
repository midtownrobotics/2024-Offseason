package frc.robot.subsystems.Limelight.LimelightIO;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class LimelightIOInputsAutoLogged extends LimelightIO.LimelightIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("AprilTagID", aprilTagID);
    table.put("ValidTargetExists", validTargetExists);
    table.put("AprilTagXCameraSpace", aprilTagXCameraSpace);
    table.put("AprilTagZCameraSpace", aprilTagZCameraSpace);
    table.put("Distance", distance);
    table.put("AngleOffset", angleOffset);
    table.put("Tx", tx);
    table.put("LatestPose", latestPose);
  }

  @Override
  public void fromLog(LogTable table) {
    aprilTagID = table.get("AprilTagID", aprilTagID);
    validTargetExists = table.get("ValidTargetExists", validTargetExists);
    aprilTagXCameraSpace = table.get("AprilTagXCameraSpace", aprilTagXCameraSpace);
    aprilTagZCameraSpace = table.get("AprilTagZCameraSpace", aprilTagZCameraSpace);
    distance = table.get("Distance", distance);
    angleOffset = table.get("AngleOffset", angleOffset);
    tx = table.get("Tx", tx);
    latestPose = table.get("LatestPose", latestPose);
  }

  public LimelightIOInputsAutoLogged clone() {
    LimelightIOInputsAutoLogged copy = new LimelightIOInputsAutoLogged();
    copy.aprilTagID = this.aprilTagID;
    copy.validTargetExists = this.validTargetExists;
    copy.aprilTagXCameraSpace = this.aprilTagXCameraSpace;
    copy.aprilTagZCameraSpace = this.aprilTagZCameraSpace;
    copy.distance = this.distance;
    copy.angleOffset = this.angleOffset;
    copy.tx = this.tx;
    copy.latestPose = this.latestPose;
    return copy;
  }
}
