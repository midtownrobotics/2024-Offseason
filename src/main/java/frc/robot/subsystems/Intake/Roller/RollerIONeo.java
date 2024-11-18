package frc.robot.subsystems.Intake.Roller;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.MotorConstants;
import frc.robot.utils.TempuratureConverter;
import org.littletonrobotics.junction.Logger;

public class RollerIONeo implements RollerIO {

  private SparkMax runExternal;
  private SparkMax runInternal;

  public RollerIONeo(int runExternalID, int runInternalID) {
    runExternal = new SparkMax(runExternalID, MotorType.kBrushless);
    runInternal = new SparkMax(runInternalID, MotorType.kBrushless);

    SparkMaxConfig externalConfig = new SparkMaxConfig();
    externalConfig.inverted(true);
    externalConfig.smartCurrentLimit(MotorConstants.CURRENT_LIMIT_550);
    runExternal.configure(externalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig internalConfig = new SparkMaxConfig();
    internalConfig.inverted(false);
    internalConfig.smartCurrentLimit(MotorConstants.CURRENT_LIMIT_550);
    runInternal.configure(internalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setSpeed(double speed) {
    runExternal.set(speed);
    runInternal.set(speed);
    Logger.recordOutput("Intake/DesiredSpeed", speed);
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    inputs.externalOutputVoltage = runExternal.getBusVoltage() * runExternal.getAppliedOutput();
    inputs.internalOutputVoltage = runInternal.getBusVoltage() * runInternal.getAppliedOutput();
    inputs.externalIsOn = Math.abs(runExternal.getAppliedOutput()) > 0.01;
    inputs.internalIsOn = Math.abs(runInternal.getAppliedOutput()) > 0.01;
    inputs.externalVelocityRPM = runExternal.getEncoder().getVelocity();
    inputs.internalVelocityRPM = runInternal.getEncoder().getVelocity();
    inputs.externalTempFahrenheit =
        TempuratureConverter.celsiusToFahrenheit(runExternal.getMotorTemperature());
    inputs.internalTempFahrenheit =
        TempuratureConverter.celsiusToFahrenheit(runInternal.getMotorTemperature());
    inputs.externalCurrentAmps = runExternal.getOutputCurrent();
    inputs.internalCurrentAmps = runInternal.getOutputCurrent();
  }
}
