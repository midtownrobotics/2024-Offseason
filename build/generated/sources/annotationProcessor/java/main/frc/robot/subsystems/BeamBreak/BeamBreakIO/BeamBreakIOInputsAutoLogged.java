package frc.robot.subsystems.BeamBreak.BeamBreakIO;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class BeamBreakIOInputsAutoLogged extends BeamBreakIO.BeamBreakIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("IsBroken", isBroken);
  }

  @Override
  public void fromLog(LogTable table) {
    isBroken = table.get("IsBroken", isBroken);
  }

  public BeamBreakIOInputsAutoLogged clone() {
    BeamBreakIOInputsAutoLogged copy = new BeamBreakIOInputsAutoLogged();
    copy.isBroken = this.isBroken;
    return copy;
  }
}
