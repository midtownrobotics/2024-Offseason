package frc.robot.subsystems.Limelight;

import edu.wpi.first.math.geometry.Pose2d;
 import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Limelight.LimelightIO.LimelightIO;
import frc.robot.subsystems.Limelight.LimelightIO.LimelightIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class Limelight extends SubsystemBase {
  private LimelightIO limelightFrontIO;
  private LimelightIO limelightBackIO;
  private LimelightIOInputsAutoLogged limelightFrontIOInputs = new LimelightIOInputsAutoLogged();
  private LimelightIOInputsAutoLogged limelightBackIOInputs = new LimelightIOInputsAutoLogged();
  private Pose2d latestVisionPose;
  private boolean autonVisionEnabled = false;

  private double lastKnownTxFront = 0;
  private double lastKnownTxFrontTime = 0;

  public Limelight(LimelightIO limelightFrontIO, LimelightIO limelightBackIO) {
    this.limelightFrontIO = limelightFrontIO;
    this.limelightBackIO = limelightBackIO;
  }

  public double getAngleOffset() {
    return limelightBackIO.getAngleOffset();
  }

  public double getDistance() {
    return limelightBackIO.getDistance();
  }

  public double getTxFront() {
    if (limelightFrontIO.getTx() != 0) {
      return -limelightFrontIO.getTx();
    } else  {
      return -lastKnownTxFront;
    }
  }

  public double getTxBack() {
    return limelightBackIO.getTx();
  }

  public boolean isValidTargetAprilTag(int targetId) {
    return limelightBackIO.getValidTargetExists() && limelightBackIO.getId() == targetId;
  }

  public boolean isValidTargetNote() {
    return lastKnownTxFront != 0;
  }
  
  @Override
  public void periodic() {
    if (limelightFrontIO.getTx() != 0) {
      lastKnownTxFront = limelightFrontIO.getTx();
      lastKnownTxFrontTime = Timer.getFPGATimestamp();
    }

    if (Timer.getFPGATimestamp() - lastKnownTxFrontTime > 0.5) {
      lastKnownTxFront = 0;
    }

    limelightFrontIO.updateInputs(limelightFrontIOInputs);
    Logger.processInputs("Limelight", limelightFrontIOInputs);

    limelightBackIO.updateInputs(limelightBackIOInputs);
    limelightBackIOInputs.latestPose = latestVisionPose;
    Logger.processInputs("Limelight", limelightBackIOInputs);

    Logger.recordOutput("Limelight/AutonVision", autonVisionEnabled);
    Logger.recordOutput("Limelight/lastKnownTx", lastKnownTxFront);
    Logger.recordOutput("Limelight/lastKnownTxTime", lastKnownTxFrontTime);
    Logger.recordOutput("Limelight/time", Timer.getFPGATimestamp());
  }

  public void setAutonVisionEnabled(boolean autonVisionEnabled) {
    this.autonVisionEnabled = autonVisionEnabled;
  }

  public LimelightHelpers.PoseEstimate getMegatagPose(Pose2d estimatedPose) {
    LimelightHelpers.SetRobotOrientation(
        "limelight", estimatedPose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2 =
        LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

    if (RobotState.isAutonomous() && !autonVisionEnabled) return null; // For now ignore vision in auto

    if (mt2 == null || mt2.pose == null || mt2.tagCount == 0) {
      Pose2d nullPose = null;
      Logger.recordOutput("Limelight/VisionPose", nullPose);
      return null;
    }

    // For some reason we need to invert?? MUST BE LOOKED INTO
    // mt2.pose =
    //     new Pose2d(new Translation2d(-mt2.pose.getX(), mt2.pose.getY()), mt2.pose.getRotation());
    latestVisionPose = mt2.pose;

    return mt2;
  }
}
