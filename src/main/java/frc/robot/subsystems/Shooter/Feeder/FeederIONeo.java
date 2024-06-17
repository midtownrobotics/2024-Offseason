package frc.robot.subsystems.Shooter.Feeder;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class FeederIONeo implements FeederIO {

    private CANSparkMax rollerTopNeo;
    private CANSparkMax rollerBottomNeo;

    public FeederIONeo(int rollerTopID, int rollerBottomID) {
        rollerTopNeo = new CANSparkMax(rollerTopID, MotorType.kBrushless);
        rollerTopNeo.setIdleMode(IdleMode.kCoast);
        rollerTopNeo.burnFlash();

        rollerBottomNeo = new CANSparkMax(rollerBottomID, MotorType.kBrushless);
        rollerBottomNeo.setIdleMode(IdleMode.kCoast);
        rollerBottomNeo.burnFlash();
    }

    @Override
    public void setVoltage(double voltage) {
        rollerTopNeo.setVoltage(voltage);
        rollerBottomNeo.setVoltage(voltage);
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        
    }
}