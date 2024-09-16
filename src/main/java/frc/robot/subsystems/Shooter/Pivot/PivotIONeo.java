package frc.robot.subsystems.Shooter.Pivot;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.TempuratureConverter;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class PivotIONeo implements PivotIO {
    private CANSparkMax pivotNeo;
    private PIDController pivotPID;
    private DutyCycleEncoder pivotEncoder;

    public PivotIONeo(int pivotNeoID, int pivotEncoderDIOID) {
        pivotNeo = new CANSparkMax(pivotNeoID, MotorType.kBrushless);
        pivotNeo.setIdleMode(IdleMode.kCoast);
        pivotNeo.setSmartCurrentLimit(MotorConstants.CURRENT_LIMIT_1650);
        pivotNeo.burnFlash();
        
        pivotEncoder = new DutyCycleEncoder(new DigitalInput(pivotEncoderDIOID));

        pivotPID = new PIDController(ShooterConstants.PIVOT_P.get(), ShooterConstants.PIVOT_I.get(), ShooterConstants.PIVOT_D.get());
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        inputs.pivotOutputVoltage = pivotNeo.getBusVoltage() * pivotNeo.getAppliedOutput();
        inputs.pivotIsOn = Math.abs(pivotNeo.getAppliedOutput()) > 0.01;
        inputs.pivotVelocityRPM = pivotNeo.getEncoder().getVelocity();
        inputs.pivotTempFahrenheit = TempuratureConverter.celsiusToFahrenheit(pivotNeo.getMotorTemperature());
        inputs.pivotCurrentAmps = pivotNeo.getOutputCurrent();
        inputs.encoderReading = pivotEncoder.getAbsolutePosition();

        double editedEncoderReading = pivotEncoder.getAbsolutePosition();

        if (editedEncoderReading < 0.5) {
            editedEncoderReading++;
        }

        inputs.editedEncoderReading = editedEncoderReading;
    }

    @Override
    public void setAngle(double angle) {

        double encoderReading = pivotEncoder.getAbsolutePosition();

        if (encoderReading < 0.5) {
            encoderReading++;
        }

        Logger.recordOutput("Shooter/DesiredPivotAngle", angle);
        Logger.recordOutput("Shooter/PivotAngle", encoderReading);

        double pidAmount = pivotPID.calculate(encoderReading, Math.min(Math.max(angle, ShooterConstants.MIN_PIVOT_ANGLE.get()), ShooterConstants.MAX_PIVOT_ANGLE.get()));

        pidAmount*= 12; // multiply by 12 because the battery is 12 volts
        
        Logger.recordOutput("Shooter/DesiredVoltage", pidAmount);

        pivotNeo.setVoltage(pidAmount);
    }

    public void updatePIDControllers() {
        LoggedTunableNumber.ifChanged(hashCode(), () -> {
            pivotPID.setPID(ShooterConstants.PIVOT_P.get(), ShooterConstants.PIVOT_I.get(), ShooterConstants.PIVOT_D.get());
        });
    }
}
