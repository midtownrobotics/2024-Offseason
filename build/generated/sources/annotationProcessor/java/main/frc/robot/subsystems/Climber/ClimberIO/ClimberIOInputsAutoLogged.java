package frc.robot.subsystems.Climber.ClimberIO;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ClimberIOInputsAutoLogged extends ClimberIO.ClimberIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("RightOutputVoltage", rightOutputVoltage);
    table.put("LeftOutputVoltage", leftOutputVoltage);
    table.put("RightIsOn", rightIsOn);
    table.put("LeftIsOn", leftIsOn);
    table.put("RightVelocityRPM", rightVelocityRPM);
    table.put("LeftVelocityRPM", leftVelocityRPM);
    table.put("RightTempFahrenheit", rightTempFahrenheit);
    table.put("LeftTempFahrenheit", leftTempFahrenheit);
    table.put("RightCurrentAmps", rightCurrentAmps);
    table.put("LeftCurrentAmps", leftCurrentAmps);
    table.put("LeftDesiredPower", leftDesiredPower);
    table.put("RightDesiredPower", rightDesiredPower);
  }

  @Override
  public void fromLog(LogTable table) {
    rightOutputVoltage = table.get("RightOutputVoltage", rightOutputVoltage);
    leftOutputVoltage = table.get("LeftOutputVoltage", leftOutputVoltage);
    rightIsOn = table.get("RightIsOn", rightIsOn);
    leftIsOn = table.get("LeftIsOn", leftIsOn);
    rightVelocityRPM = table.get("RightVelocityRPM", rightVelocityRPM);
    leftVelocityRPM = table.get("LeftVelocityRPM", leftVelocityRPM);
    rightTempFahrenheit = table.get("RightTempFahrenheit", rightTempFahrenheit);
    leftTempFahrenheit = table.get("LeftTempFahrenheit", leftTempFahrenheit);
    rightCurrentAmps = table.get("RightCurrentAmps", rightCurrentAmps);
    leftCurrentAmps = table.get("LeftCurrentAmps", leftCurrentAmps);
    leftDesiredPower = table.get("LeftDesiredPower", leftDesiredPower);
    rightDesiredPower = table.get("RightDesiredPower", rightDesiredPower);
  }

  public ClimberIOInputsAutoLogged clone() {
    ClimberIOInputsAutoLogged copy = new ClimberIOInputsAutoLogged();
    copy.rightOutputVoltage = this.rightOutputVoltage;
    copy.leftOutputVoltage = this.leftOutputVoltage;
    copy.rightIsOn = this.rightIsOn;
    copy.leftIsOn = this.leftIsOn;
    copy.rightVelocityRPM = this.rightVelocityRPM;
    copy.leftVelocityRPM = this.leftVelocityRPM;
    copy.rightTempFahrenheit = this.rightTempFahrenheit;
    copy.leftTempFahrenheit = this.leftTempFahrenheit;
    copy.rightCurrentAmps = this.rightCurrentAmps;
    copy.leftCurrentAmps = this.leftCurrentAmps;
    copy.leftDesiredPower = this.leftDesiredPower;
    copy.rightDesiredPower = this.rightDesiredPower;
    return copy;
  }
}
