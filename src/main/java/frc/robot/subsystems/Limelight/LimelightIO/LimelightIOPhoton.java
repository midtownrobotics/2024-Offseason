package frc.robot.subsystems.Limelight.LimelightIO;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class LimelightIOPhoton implements LimelightIO {
  private final PhotonCamera m_camera;
  private final PhotonPoseEstimator m_estimator;

  public LimelightIOPhoton(String name, Transform3d transform) {
    m_camera = new PhotonCamera(name);
    var aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    m_estimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, transform);
    m_estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  @Override
  public void updateInputs(LimelightIOInputs inputs) {
    for (var result : m_camera.getAllUnreadResults()) {
      m_estimator
          .update(result)
          .ifPresent(
              estimate -> {
                inputs.latestPose = estimate.estimatedPose;
                inputs.latestTimestampSeconds = estimate.timestampSeconds;
                inputs.stdDevs = calculateStdDevs();
                inputs.aprilTagID = estimate.targetsUsed.get(0).fiducialId;
              });
    }
  }

  private double[] calculateStdDevs() {
    return new double[] {0.2, 0.2, 0.5};
  }

  @Override
  public double getAngleOffset() {
    return 0;
  }

  @Override
  public double getTx() {
    return 0;
  }

  @Override
  public double getDistance() {
    return 0;
  }

  @Override
  public int getId() {
    return 0;
  }

  @Override
  public boolean getValidTargetExists() {
    return false;
  }
}
