package frc.robot.subsystems.Shooter.Flywheel;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.ShooterConstants;

public class FlywheelIONeo implements FlywheelIO {
    private CANSparkMax leftWheelNeo;
    private CANSparkMax rightWheelNeo;
    private SparkPIDController leftWheelPID;
    private SparkPIDController rightWheelPID;

    public FlywheelIONeo(int leftWheelNeoID, int rightWheelNeoID) {

        leftWheelNeo = new CANSparkMax(leftWheelNeoID, MotorType.kBrushless);
        leftWheelNeo.setIdleMode(IdleMode.kCoast);
        leftWheelNeo.burnFlash();

        rightWheelNeo = new CANSparkMax(rightWheelNeoID, MotorType.kBrushless);
        rightWheelNeo.setIdleMode(IdleMode.kCoast);
        rightWheelNeo.burnFlash();

        leftWheelPID = leftWheelNeo.getPIDController();
        leftWheelPID.setP(ShooterConstants.FLYWHEEL_SPEED_P.get());
        leftWheelPID.setI(ShooterConstants.FLYWHEEL_SPEED_I.get());
        leftWheelPID.setD(ShooterConstants.FLYWHEEL_SPEED_D.get());
        leftWheelPID.setFF(ShooterConstants.FLYWHEEL_SPEED_FF.get());
        leftWheelPID.setOutputRange(0, 1);

        rightWheelPID = rightWheelNeo.getPIDController();
        rightWheelPID.setP(ShooterConstants.FLYWHEEL_SPEED_P.get());
        rightWheelPID.setI(ShooterConstants.FLYWHEEL_SPEED_I.get());
        rightWheelPID.setD(ShooterConstants.FLYWHEEL_SPEED_D.get());
        rightWheelPID.setFF(ShooterConstants.FLYWHEEL_SPEED_FF.get());
        rightWheelPID.setOutputRange(0, 1);
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        
    }

    @Override
    public void setSpeed(double leftSpeed, double rightSpeed) {

        Logger.recordOutput("Shooter/DesiredLeftSpeed", leftSpeed);
        Logger.recordOutput("Shooter/DesiredRightSpeed", rightSpeed);

        if (leftSpeed == 0) {
            leftWheelPID.setReference(0, ControlType.kDutyCycle);
        } else {
            leftWheelPID.setReference(leftSpeed, ControlType.kVelocity);
        }

        if (rightSpeed == 0) {
            rightWheelPID.setReference(0, ControlType.kDutyCycle);
        } else {
            rightWheelPID.setReference(rightSpeed, ControlType.kVelocity);
        }
    }
}
