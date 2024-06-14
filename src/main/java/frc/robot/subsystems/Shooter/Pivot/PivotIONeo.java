package frc.robot.subsystems.Shooter.Pivot;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class PivotIONeo implements PivotIO {
    private CANSparkMax pivotNeo;
    private PIDController pivotPID;
    private DutyCycleEncoder pivotEncoder;

    public PivotIONeo(int pivotNeoID, int pivotEncoderDIOID) {
        pivotNeo = new CANSparkMax(pivotNeoID, MotorType.kBrushless);
        pivotNeo.restoreFactoryDefaults();
        pivotNeo.setIdleMode(IdleMode.kCoast);
        pivotNeo.burnFlash();

        pivotEncoder = new DutyCycleEncoder(new DigitalInput(pivotEncoderDIOID));

        pivotPID = new PIDController(10, 0, 0);
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        
    }

    @Override
    public void setAngle(double angle) {
        double pidAmount = 0;
        if (angle > .81 && angle < .99){
            pidAmount = pivotPID.calculate(pivotEncoder.getAbsolutePosition(), angle);
            pivotNeo.set(pidAmount);
        }
    }
}
