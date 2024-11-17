package frc.robot.subsystems.Intake.Roller;

import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {
  @AutoLog
  public class RollerIOInputs {
    public double externalOutputVoltage = 0.0;
    public double internalOutputVoltage = 0.0;
    public boolean externalIsOn = false;
    public boolean internalIsOn = false;
    public double externalVelocityRPM = 0.0;
    public double internalVelocityRPM = 0.0;
    public double externalTempFahrenheit = 0.0;
    public double internalTempFahrenheit = 0.0;
    public double externalCurrentAmps = 0.0;
    public double internalCurrentAmps = 0.0;
  }

  public void setSpeed(double speed);

  public void updateInputs(RollerIOInputs inputs);
}
