package frc.robot.subsystems.Shooter.Feeder;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import frc.robot.Constants.MotorConstants;
import frc.robot.utils.TempuratureConverter;

public class FeederIONeo implements FeederIO {

    private CANSparkMax rollerTopNeo;
    private CANSparkMax rollerBottomNeo;

    public FeederIONeo(int rollerTopID, int rollerBottomID) {
        rollerTopNeo = new CANSparkMax(rollerTopID, MotorType.kBrushless);
        rollerTopNeo.setIdleMode(IdleMode.kCoast);
        rollerTopNeo.setSmartCurrentLimit(MotorConstants.CURRENT_LIMIT_550);
        rollerTopNeo.burnFlash();

        rollerBottomNeo = new CANSparkMax(rollerBottomID, MotorType.kBrushless);
        rollerBottomNeo.setIdleMode(IdleMode.kCoast);
        rollerBottomNeo.setSmartCurrentLimit(MotorConstants.CURRENT_LIMIT_550);
        rollerBottomNeo.burnFlash();
    }

    @Override
    public void setVoltage(double voltage) {
        rollerTopNeo.setVoltage(voltage);
        rollerBottomNeo.setVoltage(voltage);
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        inputs.topOutputVoltage = rollerTopNeo.getBusVoltage() * rollerTopNeo.getAppliedOutput();
        inputs.bottomOutputVoltage = rollerBottomNeo.getBusVoltage() * rollerBottomNeo.getAppliedOutput();
        inputs.topIsOn = Math.abs(rollerTopNeo.getAppliedOutput()) > 0.01;
        inputs.bottomIsOn = Math.abs(rollerBottomNeo.getAppliedOutput()) > 0.01;
        inputs.topVelocityRPM = rollerTopNeo.getEncoder().getVelocity();
        inputs.bottomVelocityRPM = rollerBottomNeo.getEncoder().getVelocity();
        inputs.topTempFahrenheit = TempuratureConverter.celsiusToFahrenheit(rollerTopNeo.getMotorTemperature());
        inputs.bottomTempFahrenheit = TempuratureConverter.celsiusToFahrenheit(rollerBottomNeo.getMotorTemperature());
        inputs.topCurrentAmps = rollerTopNeo.getOutputCurrent();
        inputs.bottomCurrentAmps = rollerBottomNeo.getOutputCurrent();
    }
}