package frc.robot.subsystems.Shooter.Flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
    @AutoLog
    public class FlywheelIOInputs {

    }

    public void setSpeed(double leftSpeed, double rightSpeed);
    public void updateInputs(FlywheelIOInputs inputs);
}