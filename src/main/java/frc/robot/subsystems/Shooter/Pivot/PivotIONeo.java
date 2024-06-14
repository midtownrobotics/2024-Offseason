package frc.robot.subsystems.Shooter.Pivot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class PivotIONeo implements PivotIO {
    CANSparkMax pivotNeo;

    public PivotIONeo(int pivotNeoID) {
        pivotNeo = new CANSparkMax(pivotNeoID, MotorType.kBrushless);
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        
    }

    @Override
    public void setVoltage(double voltage) {
        pivotNeo.setVoltage(voltage);
    }
}
