package frc.robot.subsystems.Climber.ClimberIO;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.MotorConstants;
import frc.robot.utils.TempuratureConverter;

public class ClimberIONeo implements ClimberIO {
  private SparkMax rightClimber;
  private SparkMax leftClimber;

  private double leftDesiredPower;
  private double rightDesiredPower;

  public ClimberIONeo(int rightClimberID, int leftClimberID) {
    rightClimber = new SparkMax(rightClimberID, MotorType.kBrushless);
    leftClimber = new SparkMax(leftClimberID, MotorType.kBrushless);

    SparkBaseConfig rightConfig =
        new SparkMaxConfig().smartCurrentLimit(MotorConstants.CURRENT_LIMIT_1650);
    SparkBaseConfig leftConfig =
        new SparkMaxConfig().smartCurrentLimit(MotorConstants.CURRENT_LIMIT_1650);

    rightClimber.configure(
        rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftClimber.configure(
        leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setPower(double rightPower, double leftPower) {
    leftDesiredPower = leftPower;
    rightDesiredPower = rightPower;
    rightClimber.set(rightPower);
    leftClimber.set(leftPower);
  }

  public void updateInputs(ClimberIOInputs inputs) {
    inputs.leftOutputVoltage = leftClimber.getBusVoltage() * leftClimber.getAppliedOutput();
    inputs.rightOutputVoltage = rightClimber.getBusVoltage() * rightClimber.getAppliedOutput();
    inputs.leftCurrentAmps = leftClimber.getOutputCurrent();
    inputs.rightCurrentAmps = rightClimber.getOutputCurrent();
    inputs.leftIsOn = Math.abs(leftClimber.getAppliedOutput()) > 0.01;
    inputs.rightIsOn = Math.abs(rightClimber.getAppliedOutput()) > 0.01;
    inputs.leftTempFahrenheit =
        TempuratureConverter.celsiusToFahrenheit(leftClimber.getMotorTemperature());
    inputs.rightTempFahrenheit =
        TempuratureConverter.celsiusToFahrenheit(rightClimber.getMotorTemperature());
    inputs.leftVelocityRPM = leftClimber.getEncoder().getVelocity();
    inputs.rightVelocityRPM = leftClimber.getEncoder().getVelocity();

    inputs.leftDesiredPower = leftDesiredPower;
    inputs.rightDesiredPower = rightDesiredPower;
  }
}
