package frc.robot.subsystems.Shooter.Pivot;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class PivotIOInputsAutoLogged extends PivotIO.PivotIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("PivotOutputVoltage", pivotOutputVoltage);
    table.put("PivotIsOn", pivotIsOn);
    table.put("PivotVelocityRPM", pivotVelocityRPM);
    table.put("PivotTempFahrenheit", pivotTempFahrenheit);
    table.put("PivotCurrentAmps", pivotCurrentAmps);
    table.put("EncoderReading", encoderReading);
    table.put("EditedEncoderReading", editedEncoderReading);
  }

  @Override
  public void fromLog(LogTable table) {
    pivotOutputVoltage = table.get("PivotOutputVoltage", pivotOutputVoltage);
    pivotIsOn = table.get("PivotIsOn", pivotIsOn);
    pivotVelocityRPM = table.get("PivotVelocityRPM", pivotVelocityRPM);
    pivotTempFahrenheit = table.get("PivotTempFahrenheit", pivotTempFahrenheit);
    pivotCurrentAmps = table.get("PivotCurrentAmps", pivotCurrentAmps);
    encoderReading = table.get("EncoderReading", encoderReading);
    editedEncoderReading = table.get("EditedEncoderReading", editedEncoderReading);
  }

  public PivotIOInputsAutoLogged clone() {
    PivotIOInputsAutoLogged copy = new PivotIOInputsAutoLogged();
    copy.pivotOutputVoltage = this.pivotOutputVoltage;
    copy.pivotIsOn = this.pivotIsOn;
    copy.pivotVelocityRPM = this.pivotVelocityRPM;
    copy.pivotTempFahrenheit = this.pivotTempFahrenheit;
    copy.pivotCurrentAmps = this.pivotCurrentAmps;
    copy.encoderReading = this.encoderReading;
    copy.editedEncoderReading = this.editedEncoderReading;
    return copy;
  }
}
