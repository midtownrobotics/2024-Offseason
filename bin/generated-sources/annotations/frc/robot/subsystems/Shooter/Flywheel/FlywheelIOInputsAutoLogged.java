package frc.robot.subsystems.Shooter.Flywheel;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class FlywheelIOInputsAutoLogged extends FlywheelIO.FlywheelIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("LeftOutputVoltage", leftOutputVoltage);
    table.put("RightOutputVoltage", rightOutputVoltage);
    table.put("LeftIsOn", leftIsOn);
    table.put("RightIsOn", rightIsOn);
    table.put("LeftVelocityRPM", leftVelocityRPM);
    table.put("RightVelocityRPM", rightVelocityRPM);
    table.put("LeftTempFahrenheit", leftTempFahrenheit);
    table.put("RightTempFahrenheit", rightTempFahrenheit);
    table.put("LeftCurrentAmps", leftCurrentAmps);
    table.put("RightCurrentAmps", rightCurrentAmps);
  }

  @Override
  public void fromLog(LogTable table) {
    leftOutputVoltage = table.get("LeftOutputVoltage", leftOutputVoltage);
    rightOutputVoltage = table.get("RightOutputVoltage", rightOutputVoltage);
    leftIsOn = table.get("LeftIsOn", leftIsOn);
    rightIsOn = table.get("RightIsOn", rightIsOn);
    leftVelocityRPM = table.get("LeftVelocityRPM", leftVelocityRPM);
    rightVelocityRPM = table.get("RightVelocityRPM", rightVelocityRPM);
    leftTempFahrenheit = table.get("LeftTempFahrenheit", leftTempFahrenheit);
    rightTempFahrenheit = table.get("RightTempFahrenheit", rightTempFahrenheit);
    leftCurrentAmps = table.get("LeftCurrentAmps", leftCurrentAmps);
    rightCurrentAmps = table.get("RightCurrentAmps", rightCurrentAmps);
  }

  public FlywheelIOInputsAutoLogged clone() {
    FlywheelIOInputsAutoLogged copy = new FlywheelIOInputsAutoLogged();
    copy.leftOutputVoltage = this.leftOutputVoltage;
    copy.rightOutputVoltage = this.rightOutputVoltage;
    copy.leftIsOn = this.leftIsOn;
    copy.rightIsOn = this.rightIsOn;
    copy.leftVelocityRPM = this.leftVelocityRPM;
    copy.rightVelocityRPM = this.rightVelocityRPM;
    copy.leftTempFahrenheit = this.leftTempFahrenheit;
    copy.rightTempFahrenheit = this.rightTempFahrenheit;
    copy.leftCurrentAmps = this.leftCurrentAmps;
    copy.rightCurrentAmps = this.rightCurrentAmps;
    return copy;
  }
}
