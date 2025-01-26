package frc.robot.subsystems.Limelight;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Limelight.LimelightIO.LimelightIO;
import frc.robot.subsystems.Limelight.LimelightIO.LimelightIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class Limelight extends SubsystemBase {
  private LimelightIO limelightIO;
  private LimelightIOInputsAutoLogged limelightIOInputs = new LimelightIOInputsAutoLogged();
  private Pose2d latestVisionPose;
  private boolean autonVisionEnabled = false;

  public Limelight(LimelightIO limelightIO) {
    this.limelightIO = limelightIO;
    limelightIOInputs.stdDevs = new double[] {1.2, 1.2, 5};
  }

  public double getAngleOffset() {
    return limelightIO.getAngleOffset();
  }

  public double getDistance() {
    return limelightIO.getDistance();
  }

  public double getTx() {
    return limelightIO.getTx();
  }

  public boolean isValidTarget(int targetId) {
    return limelightIO.getValidTargetExists() && limelightIO.getId() == targetId;
  }

  @Override
  public void periodic() {
    limelightIO.updateInputs(limelightIOInputs);
    if (latestVisionPose != null) {
      limelightIOInputs.latestPose = new Pose3d(latestVisionPose);
    }
    Logger.processInputs("Limelight", limelightIOInputs);
    Logger.recordOutput("Limelight/AutonVision", autonVisionEnabled);
  }

  public void setAutonVisionEnabled(boolean autonVisionEnabled) {
    this.autonVisionEnabled = autonVisionEnabled;
  }

  public Pose3d getLatestPose() {
    return limelightIOInputs.latestPose;
  }

  public double getLatestTimestamp() {
    return limelightIOInputs.latestTimestampSeconds;
  }

  public Vector<N3> getLatestStdDevs() {
    double xStdDev = limelightIOInputs.stdDevs[0];
    double yStdDev = limelightIOInputs.stdDevs[1];
    double rotStdDev = limelightIOInputs.stdDevs[2];

    return VecBuilder.fill(xStdDev, yStdDev, rotStdDev);
  }

  public LimelightHelpers.PoseEstimate getMegatagPose(Pose2d estimatedPose) {
    LimelightHelpers.SetRobotOrientation(
        "limelight-woody", estimatedPose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2 =
        LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-woody");

    if (mt2 == null || mt2.pose == null || mt2.tagCount == 0) {
      Pose2d nullPose = null;
      Logger.recordOutput("Limelight/VisionPose", nullPose);
      return null;
    }

    Logger.recordOutput("Limelight/VisionPose", mt2.pose);
    latestVisionPose = mt2.pose;

    return mt2;
  }
}
