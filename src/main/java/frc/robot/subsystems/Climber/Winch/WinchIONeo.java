package frc.robot.subsystems.Climber.Winch;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.ClimberConstants;

public class WinchIONeo implements WinchIO {
    private CANSparkMax leftWinchNeo;
    private CANSparkMax rightWinchNeo;
    private SparkPIDController leftWinchPID;
    private SparkPIDController rightWinchPID;

    public WinchIONeo(int leftWinchNeoID, int rightWinchNeoID) {
        leftWinchNeo = new CANSparkMax(leftWinchNeoID, MotorType.kBrushless);
        leftWinchNeo.restoreFactoryDefaults();
        leftWinchNeo.setIdleMode(IdleMode.kBrake);
        leftWinchNeo.burnFlash();

        rightWinchNeo = new CANSparkMax(rightWinchNeoID, MotorType.kBrushless);
        rightWinchNeo.restoreFactoryDefaults();
        rightWinchNeo.setIdleMode(IdleMode.kBrake);
        rightWinchNeo.burnFlash();

        leftWinchPID = leftWinchNeo.getPIDController();
        leftWinchPID.setP(ClimberConstants.WINCH_P);
        leftWinchPID.setI(ClimberConstants.WINCH_I);
        leftWinchPID.setD(ClimberConstants.WINCH_D);
        leftWinchPID.setFF(ClimberConstants.WINCH_FF);
        leftWinchPID.setOutputRange(0, 1);

        rightWinchPID = rightWinchNeo.getPIDController();
        rightWinchPID.setP(ClimberConstants.WINCH_P);
        rightWinchPID.setI(ClimberConstants.WINCH_I);
        rightWinchPID.setD(ClimberConstants.WINCH_D);
        rightWinchPID.setFF(ClimberConstants.WINCH_FF);
        rightWinchPID.setOutputRange(0, 1);
    }
   
    public void updateInputs(WinchIOInputs inputs) {

    }

    public void setSpeed(double leftSpeed, double rightSpeed){
       leftWinchPID.setReference(leftSpeed, ControlType.kVelocity);
       rightWinchPID.setReference(rightSpeed, ControlType.kVelocity);
    }
}
