package frc.robot.subsystems.Shooter.Roller;

import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {
    @AutoLog
    public class RollerIOInputs {

    }
    
    public void setVoltage(double voltage);
    public void updateInputs(RollerIOInputs inputs);
}
