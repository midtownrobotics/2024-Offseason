package frc.robot.subsystems.Shooter.Pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
    @AutoLog
    public class PivotIOInputs {

    }
    
    public void setVoltage(double voltage);
    public void updateInputs(PivotIOInputs inputs);
}
