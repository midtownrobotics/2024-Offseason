package frc.robot.subsystems.Limelight.LimelightIO;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface LimelightIO {
  @AutoLog
  public class LimelightIOInputs {
    public int aprilTagID = 0;
    public boolean validTargetExists = false;
    public double aprilTagXCameraSpace = 0.0;
    public double aprilTagZCameraSpace = 0.0;
    public double distance = 0.0;
    public double angleOffset = 0.0;
    public double tx = 0;
    public Pose3d latestPose = new Pose3d();
    public double latestTimestampSeconds = 0.0;
    public double[] stdDevs = {0, 0, 0};
  }

  double getAngleOffset();

  double getTx();

  double getDistance();

  int getId();

  boolean getValidTargetExists();

  void updateInputs(LimelightIOInputs inputs);
}
