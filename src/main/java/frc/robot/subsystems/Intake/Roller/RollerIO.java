package frc.robot.subsystems.Intake.Roller;

import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {
    @AutoLog
    public class RollerIOInputs {

    }
    
    public void setSpeed(double speed);
    public void updateInputs(RollerIOInputs inputs);
}
