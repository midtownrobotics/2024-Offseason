package frc.robot.subsystems.Limelight;

import java.util.Arrays;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Limelight.LimelightIO.LimelightIO;
import frc.robot.subsystems.Limelight.LimelightIO.LimelightIOInputsAutoLogged;

public class Limelight extends SubsystemBase{
   private LimelightIO limelightIO;
   private LimelightIOInputsAutoLogged limelightIOInputs = new LimelightIOInputsAutoLogged();

   public Limelight(LimelightIO limelightIO) {
        this.limelightIO = limelightIO;
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
     Logger.processInputs("Limelight", limelightIOInputs);
   }

   public LimelightHelpers.PoseEstimate getMegatagPose(Pose2d estimatedPose) {
     LimelightHelpers.SetRobotOrientation("limelight", estimatedPose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
     LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
     if (mt2 == null || mt2.pose == null || mt2.tagCount == 0) {
          Pose2d nullPose = null;
          Logger.recordOutput("Limelight/VisionPose", nullPose);
          return null;
     }
     
     // For some reason we need to invert?? MUST BE LOOKED INTO
     mt2.pose = new Pose2d(new Translation2d(-mt2.pose.getX(), mt2.pose.getY()), mt2.pose.getRotation());
     Logger.recordOutput("Limelight/VisionPose", mt2.pose);

     return mt2;
}
   
}
