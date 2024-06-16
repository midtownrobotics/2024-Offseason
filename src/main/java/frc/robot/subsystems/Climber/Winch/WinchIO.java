package frc.robot.subsystems.Climber.Winch;

public interface WinchIO {
   public class WinchIOInputs {

   }
   
   public void setSpeed(double leftSpeed, double rightSpeed);
   public void updateInputs(WinchIOInputs inputs);
}
