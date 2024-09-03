package frc.robot.subsystems.Limelight.LimelightIO;

import org.littletonrobotics.junction.AutoLog;

public interface LimelightIO {
    @AutoLog
    public class LimelighIOInputs {
        public int aprilTagID;
        public boolean validTargetExists;
        public double aprilTagXCameraSpace; 
        public double aprilTagZCameraSpace; 
    }

   double getAngleOffset();
   double getDistance();
   void updateInputs(LimelighIOInputs inputs);
}
