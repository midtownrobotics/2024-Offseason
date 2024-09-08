package frc.robot.subsystems.Shooter.Pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
    @AutoLog
    public class PivotIOInputs {
        public double pivotOutputVoltage = 0.0;
        public boolean pivotIsOn = false;
        public double pivotVelocityRPM = 0.0;
        public double pivotTempFahrenheit = 0.0;
        public double pivotCurrentAmps = 0.0;
        public double encoderReading = 0.0;
        public double editedEncoderReading = 0.0;
    }
    
    public void setAngle(double angle);
    public void updateInputs(PivotIOInputs inputs);
    public void updatePIDControllers();
}
