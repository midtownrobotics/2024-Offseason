package frc.robot.subsystems.Shooter.Flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
    @AutoLog
    public class FlywheelIOInputs {
        public double leftOutputVoltage = 0.0;
        public double rightOutputVoltage = 0.0;
        public boolean leftIsOn = false;
        public boolean rightIsOn = false;
        public double leftVelocityRPM = 0.0;
        public double rightVelocityRPM = 0.0;
        public double leftTempFahrenheit = 0.0;
        public double rightTempFahrenheit = 0.0;
        public double leftCurrentAmps = 0.0;
        public double rightCurrentAmps = 0.0;
    }

    public void setSpeed(double leftSpeed, double rightSpeed);
    public void updateInputs(FlywheelIOInputs inputs);
}