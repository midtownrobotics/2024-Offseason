package frc.robot.subsystems.Shooter.Flywheel;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class FlywheelIONeo implements FlywheelIO {
    CANSparkMax leftWheelNeo;
    CANSparkMax rightWheelNeo;

    public FlywheelIONeo(int leftWheelNeoID, int rightWheelNeoID) {
        leftWheelNeo = new CANSparkMax(leftWheelNeoID, MotorType.kBrushless);
        rightWheelNeo = new CANSparkMax(rightWheelNeoID, MotorType.kBrushless);
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        
    }

    @Override
    public void setSpeed(double leftSpeed, double rightSpeed) {
        leftWheelNeo.setVoltage(leftVoltage);
        rightWheelNeo.setVoltage(rightVoltage);
    }
}
