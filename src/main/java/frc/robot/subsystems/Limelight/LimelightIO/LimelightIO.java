package frc.robot.subsystems.Limelight.LimelightIO;

import org.littletonrobotics.junction.AutoLog;

public interface LimelightIO {
    @AutoLog
    public class LimelightIOInputs {
        public int aprilTagID = 0;
        public boolean validTargetExists = false;
        public double aprilTagXCameraSpace = 0.0; 
        public double aprilTagZCameraSpace = 0.0; 
    }

   double getAngleOffset();
   double getDistance();
   void updateInputs(LimelightIOInputs inputs);
}
