package frc.robot.subsystems.Shooter.Feeder;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.MotorConstants;
import frc.robot.utils.TempuratureConverter;

public class FeederIONeo implements FeederIO {

  private SparkMax rollerTopNeo;
  private SparkMax rollerBottomNeo;

  public FeederIONeo(int rollerTopID, int rollerBottomID) {
    SparkBaseConfig rollerTopConfig =
        new SparkMaxConfig()
            .idleMode(IdleMode.kCoast)
            .inverted(true)
            .smartCurrentLimit(MotorConstants.CURRENT_LIMIT_550);
    SparkBaseConfig rollerBottomConfig =
        new SparkMaxConfig()
            .idleMode(IdleMode.kCoast)
            .inverted(true)
            .smartCurrentLimit(MotorConstants.CURRENT_LIMIT_550);

    rollerTopNeo = new SparkMax(rollerTopID, MotorType.kBrushless);
    rollerTopNeo.configure(
        rollerTopConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    rollerBottomNeo = new SparkMax(rollerBottomID, MotorType.kBrushless);
    rollerBottomNeo.configure(
        rollerBottomConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setVoltage(double voltage) {
    rollerTopNeo.setVoltage(voltage);
    rollerBottomNeo.setVoltage(voltage);
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    inputs.topOutputVoltage = rollerTopNeo.getBusVoltage() * rollerTopNeo.getAppliedOutput();
    inputs.bottomOutputVoltage =
        rollerBottomNeo.getBusVoltage() * rollerBottomNeo.getAppliedOutput();
    inputs.topIsOn = Math.abs(rollerTopNeo.getAppliedOutput()) > 0.01;
    inputs.bottomIsOn = Math.abs(rollerBottomNeo.getAppliedOutput()) > 0.01;
    inputs.topVelocityRPM = rollerTopNeo.getEncoder().getVelocity();
    inputs.bottomVelocityRPM = rollerBottomNeo.getEncoder().getVelocity();
    inputs.topTempFahrenheit =
        TempuratureConverter.celsiusToFahrenheit(rollerTopNeo.getMotorTemperature());
    inputs.bottomTempFahrenheit =
        TempuratureConverter.celsiusToFahrenheit(rollerBottomNeo.getMotorTemperature());
    inputs.topCurrentAmps = rollerTopNeo.getOutputCurrent();
    inputs.bottomCurrentAmps = rollerBottomNeo.getOutputCurrent();
  }
}
