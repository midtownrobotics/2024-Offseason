package frc.robot.subsystems.Intake.Roller;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class RollerIOInputsAutoLogged extends RollerIO.RollerIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("ExternalOutputVoltage", externalOutputVoltage);
    table.put("InternalOutputVoltage", internalOutputVoltage);
    table.put("ExternalIsOn", externalIsOn);
    table.put("InternalIsOn", internalIsOn);
    table.put("ExternalVelocityRPM", externalVelocityRPM);
    table.put("InternalVelocityRPM", internalVelocityRPM);
    table.put("ExternalTempFahrenheit", externalTempFahrenheit);
    table.put("InternalTempFahrenheit", internalTempFahrenheit);
    table.put("ExternalCurrentAmps", externalCurrentAmps);
    table.put("InternalCurrentAmps", internalCurrentAmps);
  }

  @Override
  public void fromLog(LogTable table) {
    externalOutputVoltage = table.get("ExternalOutputVoltage", externalOutputVoltage);
    internalOutputVoltage = table.get("InternalOutputVoltage", internalOutputVoltage);
    externalIsOn = table.get("ExternalIsOn", externalIsOn);
    internalIsOn = table.get("InternalIsOn", internalIsOn);
    externalVelocityRPM = table.get("ExternalVelocityRPM", externalVelocityRPM);
    internalVelocityRPM = table.get("InternalVelocityRPM", internalVelocityRPM);
    externalTempFahrenheit = table.get("ExternalTempFahrenheit", externalTempFahrenheit);
    internalTempFahrenheit = table.get("InternalTempFahrenheit", internalTempFahrenheit);
    externalCurrentAmps = table.get("ExternalCurrentAmps", externalCurrentAmps);
    internalCurrentAmps = table.get("InternalCurrentAmps", internalCurrentAmps);
  }

  public RollerIOInputsAutoLogged clone() {
    RollerIOInputsAutoLogged copy = new RollerIOInputsAutoLogged();
    copy.externalOutputVoltage = this.externalOutputVoltage;
    copy.internalOutputVoltage = this.internalOutputVoltage;
    copy.externalIsOn = this.externalIsOn;
    copy.internalIsOn = this.internalIsOn;
    copy.externalVelocityRPM = this.externalVelocityRPM;
    copy.internalVelocityRPM = this.internalVelocityRPM;
    copy.externalTempFahrenheit = this.externalTempFahrenheit;
    copy.internalTempFahrenheit = this.internalTempFahrenheit;
    copy.externalCurrentAmps = this.externalCurrentAmps;
    copy.internalCurrentAmps = this.internalCurrentAmps;
    return copy;
  }
}
