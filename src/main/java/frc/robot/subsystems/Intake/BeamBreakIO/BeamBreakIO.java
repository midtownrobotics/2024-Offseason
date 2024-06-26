package frc.robot.subsystems.Intake.BeamBreakIO;

import org.littletonrobotics.junction.AutoLog;

public interface BeamBreakIO {
    @AutoLog
    public class BeamBreakIOInputs {
        public boolean isBroken = false;
    }

    public void updateInputs(BeamBreakIOInputs inputs);

    public boolean getIsBroken();
}
