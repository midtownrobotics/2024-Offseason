package frc.robot.subsystems.Climber.ClimberIO;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

  @AutoLog
  public class ClimberIOInputs {
    public double rightOutputVoltage = 0.0;
    public double leftOutputVoltage = 0.0;
    public boolean rightIsOn = false;
    public boolean leftIsOn = false;
    public double rightVelocityRPM = 0.0;
    public double leftVelocityRPM = 0.0;
    public double rightTempFahrenheit = 0.0;
    public double leftTempFahrenheit = 0.0;
    public double rightCurrentAmps = 0.0;
    public double leftCurrentAmps = 0.0;
    public double leftDesiredPower;
    public double rightDesiredPower;
  }

  public void setPower(double rightPower, double leftPower);

  public void updateInputs(ClimberIOInputs inputs);
}
