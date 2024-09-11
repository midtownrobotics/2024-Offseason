package frc.robot.subsystems.Intake.Roller;

import com.revrobotics.CANSparkMax;

import frc.robot.Constants.MotorConstants;
import frc.robot.utils.TempuratureConverter;

import com.revrobotics.CANSparkLowLevel.MotorType;

public class RollerIONeo implements RollerIO {

    private CANSparkMax runExternal;
    private CANSparkMax runInternal; 

    public RollerIONeo(int runExternalID, int runInternalID) {
        runExternal = new CANSparkMax(runExternalID, MotorType.kBrushless);
        runInternal = new CANSparkMax(runInternalID, MotorType.kBrushless);

        runExternal.restoreFactoryDefaults();
        runExternal.setInverted(true);
        runExternal.setSmartCurrentLimit(MotorConstants.CURRENT_LIMIT_550);
        runExternal.burnFlash();

        runInternal.restoreFactoryDefaults();
        runInternal.setInverted(false);
        runInternal.setSmartCurrentLimit(MotorConstants.CURRENT_LIMIT_550);
        runInternal.burnFlash();
    }

    @Override
    public void setSpeed(double speed) {
        runExternal.set(speed);
        runInternal.set(speed);
    }


    @Override
    public void updateInputs(RollerIOInputs inputs) {
        inputs.externalOutputVoltage = runExternal.getBusVoltage() * runExternal.getAppliedOutput();
        inputs.internalOutputVoltage = runInternal.getBusVoltage() * runInternal.getAppliedOutput();
        inputs.externalIsOn = Math.abs(runExternal.getAppliedOutput()) > 0.01;
        inputs.internalIsOn = Math.abs(runInternal.getAppliedOutput()) > 0.01;
        inputs.externalVelocityRPM = runExternal.getEncoder().getVelocity();
        inputs.internalVelocityRPM = runInternal.getEncoder().getVelocity();
        inputs.externalTempFahrenheit = TempuratureConverter.celsiusToFahrenheit(runExternal.getMotorTemperature());
        inputs.internalTempFahrenheit = TempuratureConverter.celsiusToFahrenheit(runInternal.getMotorTemperature());
        inputs.externalCurrentAmps = runExternal.getOutputCurrent();
        inputs.internalCurrentAmps = runInternal.getOutputCurrent();
    }
}
