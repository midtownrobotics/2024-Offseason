package frc.robot.subsystems.Climber.ClimberIO;

import frc.robot.Constants.MotorConstants;
import frc.robot.utils.TempuratureConverter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ClimberIONeo implements ClimberIO{
    private CANSparkMax rightClimber;
    private CANSparkMax leftClimber;

    private double leftDesiredPower;
    private double rightDesiredPower;
    
    public ClimberIONeo(int rightClimberID, int leftClimberID) {
        rightClimber = new CANSparkMax(rightClimberID, MotorType.kBrushless);
        leftClimber = new CANSparkMax(leftClimberID, MotorType.kBrushless);

        rightClimber.setSmartCurrentLimit(MotorConstants.CURRENT_LIMIT_1650);
        rightClimber.burnFlash();
        leftClimber.setSmartCurrentLimit(MotorConstants.CURRENT_LIMIT_1650);
        leftClimber.burnFlash();
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
        inputs.leftTempFahrenheit = TempuratureConverter.celsiusToFahrenheit(leftClimber.getMotorTemperature());
        inputs.rightTempFahrenheit = TempuratureConverter.celsiusToFahrenheit(rightClimber.getMotorTemperature());
        inputs.leftVelocityRPM = leftClimber.getEncoder().getVelocity();
        inputs.rightVelocityRPM = leftClimber.getEncoder().getVelocity();

        inputs.leftDesiredPower = leftDesiredPower;
        inputs.rightDesiredPower = rightDesiredPower; 
    }
}
