package frc.robot.subsystems.BeamBreak.BeamBreakIO;

import org.littletonrobotics.junction.AutoLog;

public interface BeamBreakIO {
  @AutoLog
  public class BeamBreakIOInputs {
    public boolean isBroken = false;
  }

  public void updateInputs(BeamBreakIOInputs inputs);

  /**
   * returns if the beambreak sensor has been broken
   * @return true if we have a note, false if we don't
   */
  public boolean getIsBroken();
}
