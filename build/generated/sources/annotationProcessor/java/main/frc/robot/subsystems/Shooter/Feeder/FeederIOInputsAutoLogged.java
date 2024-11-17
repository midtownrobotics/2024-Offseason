package frc.robot.subsystems.Shooter.Feeder;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class FeederIOInputsAutoLogged extends FeederIO.FeederIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("TopOutputVoltage", topOutputVoltage);
    table.put("BottomOutputVoltage", bottomOutputVoltage);
    table.put("TopIsOn", topIsOn);
    table.put("BottomIsOn", bottomIsOn);
    table.put("TopVelocityRPM", topVelocityRPM);
    table.put("BottomVelocityRPM", bottomVelocityRPM);
    table.put("TopTempFahrenheit", topTempFahrenheit);
    table.put("BottomTempFahrenheit", bottomTempFahrenheit);
    table.put("TopCurrentAmps", topCurrentAmps);
    table.put("BottomCurrentAmps", bottomCurrentAmps);
  }

  @Override
  public void fromLog(LogTable table) {
    topOutputVoltage = table.get("TopOutputVoltage", topOutputVoltage);
    bottomOutputVoltage = table.get("BottomOutputVoltage", bottomOutputVoltage);
    topIsOn = table.get("TopIsOn", topIsOn);
    bottomIsOn = table.get("BottomIsOn", bottomIsOn);
    topVelocityRPM = table.get("TopVelocityRPM", topVelocityRPM);
    bottomVelocityRPM = table.get("BottomVelocityRPM", bottomVelocityRPM);
    topTempFahrenheit = table.get("TopTempFahrenheit", topTempFahrenheit);
    bottomTempFahrenheit = table.get("BottomTempFahrenheit", bottomTempFahrenheit);
    topCurrentAmps = table.get("TopCurrentAmps", topCurrentAmps);
    bottomCurrentAmps = table.get("BottomCurrentAmps", bottomCurrentAmps);
  }

  public FeederIOInputsAutoLogged clone() {
    FeederIOInputsAutoLogged copy = new FeederIOInputsAutoLogged();
    copy.topOutputVoltage = this.topOutputVoltage;
    copy.bottomOutputVoltage = this.bottomOutputVoltage;
    copy.topIsOn = this.topIsOn;
    copy.bottomIsOn = this.bottomIsOn;
    copy.topVelocityRPM = this.topVelocityRPM;
    copy.bottomVelocityRPM = this.bottomVelocityRPM;
    copy.topTempFahrenheit = this.topTempFahrenheit;
    copy.bottomTempFahrenheit = this.bottomTempFahrenheit;
    copy.topCurrentAmps = this.topCurrentAmps;
    copy.bottomCurrentAmps = this.bottomCurrentAmps;
    return copy;
  }
}
