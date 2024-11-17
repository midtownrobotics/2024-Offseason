package frc.robot.subsystems.Drivetrain.SwerveModuleIO;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class SwerveModuleIOInputsAutoLogged extends SwerveModuleIO.SwerveModuleIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("TurningCurrentAmps", turningCurrentAmps);
    table.put("TurningTempFahrenheit", turningTempFahrenheit);
    table.put("TurningVelocityRPM", turningVelocityRPM);
    table.put("TurningIsOn", turningIsOn);
    table.put("TurningVoltage", turningVoltage);
    table.put("DrivingCurrentAmps", drivingCurrentAmps);
    table.put("DrivingTempFahrenheit", drivingTempFahrenheit);
    table.put("DrivingVelocityRPM", drivingVelocityRPM);
    table.put("DrivingIsOn", drivingIsOn);
    table.put("DrivingVoltage", drivingVoltage);
    table.put("CurrentState", currentState);
    table.put("DesiredState", desiredState);
    table.put("Offset", offset);
    table.put("TurningEncoderPosition", turningEncoderPosition);
    table.put("TurningAbsolutePosition", turningAbsolutePosition);
  }

  @Override
  public void fromLog(LogTable table) {
    turningCurrentAmps = table.get("TurningCurrentAmps", turningCurrentAmps);
    turningTempFahrenheit = table.get("TurningTempFahrenheit", turningTempFahrenheit);
    turningVelocityRPM = table.get("TurningVelocityRPM", turningVelocityRPM);
    turningIsOn = table.get("TurningIsOn", turningIsOn);
    turningVoltage = table.get("TurningVoltage", turningVoltage);
    drivingCurrentAmps = table.get("DrivingCurrentAmps", drivingCurrentAmps);
    drivingTempFahrenheit = table.get("DrivingTempFahrenheit", drivingTempFahrenheit);
    drivingVelocityRPM = table.get("DrivingVelocityRPM", drivingVelocityRPM);
    drivingIsOn = table.get("DrivingIsOn", drivingIsOn);
    drivingVoltage = table.get("DrivingVoltage", drivingVoltage);
    currentState = table.get("CurrentState", currentState);
    desiredState = table.get("DesiredState", desiredState);
    offset = table.get("Offset", offset);
    turningEncoderPosition = table.get("TurningEncoderPosition", turningEncoderPosition);
    turningAbsolutePosition = table.get("TurningAbsolutePosition", turningAbsolutePosition);
  }

  public SwerveModuleIOInputsAutoLogged clone() {
    SwerveModuleIOInputsAutoLogged copy = new SwerveModuleIOInputsAutoLogged();
    copy.turningCurrentAmps = this.turningCurrentAmps;
    copy.turningTempFahrenheit = this.turningTempFahrenheit;
    copy.turningVelocityRPM = this.turningVelocityRPM;
    copy.turningIsOn = this.turningIsOn;
    copy.turningVoltage = this.turningVoltage;
    copy.drivingCurrentAmps = this.drivingCurrentAmps;
    copy.drivingTempFahrenheit = this.drivingTempFahrenheit;
    copy.drivingVelocityRPM = this.drivingVelocityRPM;
    copy.drivingIsOn = this.drivingIsOn;
    copy.drivingVoltage = this.drivingVoltage;
    copy.currentState = this.currentState;
    copy.desiredState = this.desiredState;
    copy.offset = this.offset;
    copy.turningEncoderPosition = this.turningEncoderPosition;
    copy.turningAbsolutePosition = this.turningAbsolutePosition;
    return copy;
  }
}
