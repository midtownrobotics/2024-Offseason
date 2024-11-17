package frc.robot.subsystems.Shooter.Feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
  @AutoLog
  public class FeederIOInputs {
    public double topOutputVoltage = 0.0;
    public double bottomOutputVoltage = 0.0;
    public boolean topIsOn = false;
    public boolean bottomIsOn = false;
    public double topVelocityRPM = 0.0;
    public double bottomVelocityRPM = 0.0;
    public double topTempFahrenheit = 0.0;
    public double bottomTempFahrenheit = 0.0;
    public double topCurrentAmps = 0.0;
    public double bottomCurrentAmps = 0.0;
  }

  public void setVoltage(double voltage);

  public void updateInputs(FeederIOInputs inputs);
}
