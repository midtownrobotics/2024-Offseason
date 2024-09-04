package frc.robot.subsystems.Limelight;

import frc.robot.subsystems.Limelight.LimelightIO.LimelightIO;

public class Limelight {
   private LimelightIO limelightIO;

   public double getAngleOffset() {
        return limelightIO.getAngleOffset();
   }

   public double getDistance() {
        return limelightIO.getDistance();
   }
}
