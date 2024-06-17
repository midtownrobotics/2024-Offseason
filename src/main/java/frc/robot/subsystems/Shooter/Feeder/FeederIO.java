package frc.robot.subsystems.Shooter.Feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
    @AutoLog
    public class FeederIOInputs {

    }
    
    public void setVoltage(double voltage);
    public void updateInputs(FeederIOInputs inputs);
}
