package frc.robot.subsystems.Intake.BeamBreakIO;

import org.littletonrobotics.junction.AutoLog;

public interface BeamBreakIO {
    @AutoLog
    public class BeamBreakIOInputs {
        
    }

    public boolean getIsBroken();
}
