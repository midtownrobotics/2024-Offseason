package frc.robot.subsystems.Shooter.Flywheel;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.OuttakeConstants;

public class FlywheelIONeo implements FlywheelIO {
    private CANSparkMax leftWheelNeo;
    private CANSparkMax rightWheelNeo;
    private SparkPIDController leftWheelPID;
    private SparkPIDController rightWheelPID;

    public FlywheelIONeo(int leftWheelNeoID, int rightWheelNeoID) {

        leftWheelNeo = new CANSparkMax(leftWheelNeoID, MotorType.kBrushless);
        leftWheelNeo.restoreFactoryDefaults();
        leftWheelNeo.setIdleMode(IdleMode.kCoast);
        leftWheelNeo.burnFlash();

        rightWheelNeo = new CANSparkMax(rightWheelNeoID, MotorType.kBrushless);
        rightWheelNeo.restoreFactoryDefaults();
        rightWheelNeo.setIdleMode(IdleMode.kCoast);
        rightWheelNeo.burnFlash();

        leftWheelPID = leftWheelNeo.getPIDController();
        leftWheelPID.setP(OuttakeConstants.FLYWHEEL_SPEED_P);
        leftWheelPID.setI(OuttakeConstants.FLYWHEEL_SPEED_I);
        leftWheelPID.setD(OuttakeConstants.FLYWHEEL_SPEED_D);
        leftWheelPID.setFF(OuttakeConstants.FLYWHEEL_SPEED_FF);
        leftWheelPID.setOutputRange(0, 1);

        rightWheelPID = rightWheelNeo.getPIDController();
        rightWheelPID.setP(OuttakeConstants.FLYWHEEL_SPEED_P);
        rightWheelPID.setI(OuttakeConstants.FLYWHEEL_SPEED_I);
        rightWheelPID.setD(OuttakeConstants.FLYWHEEL_SPEED_D);
        rightWheelPID.setFF(OuttakeConstants.FLYWHEEL_SPEED_FF);
        rightWheelPID.setOutputRange(0, 1);
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        
    }

    @Override
    public void setSpeed(double leftSpeed, double rightSpeed) {
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
