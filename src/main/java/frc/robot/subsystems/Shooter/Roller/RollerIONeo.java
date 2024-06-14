package frc.robot.subsystems.Shooter.Roller;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class RollerIONeo implements RollerIO {

    private CANSparkMax rollerTopNeo;
    private CANSparkMax rollerBottomNeo;

    public RollerIONeo(int rollerTopID, int rollerBottomID) {
        rollerTopNeo = new CANSparkMax(rollerTopID, MotorType.kBrushless);
        rollerTopNeo.restoreFactoryDefaults();
        rollerTopNeo.setIdleMode(IdleMode.kCoast);
        rollerTopNeo.burnFlash();

        rollerBottomNeo = new CANSparkMax(rollerBottomID, MotorType.kBrushless);
        rollerBottomNeo.restoreFactoryDefaults();
        rollerBottomNeo.setIdleMode(IdleMode.kCoast);
        rollerBottomNeo.burnFlash();
    }

    @Override
    public void setVoltage(double voltage) {
        rollerTopNeo.setVoltage(voltage);
        rollerBottomNeo.setVoltage(voltage);
    }

    @Override
    public void updateInputs(RollerIOInputs inputs) {
        
    }
}