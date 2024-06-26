package frc.robot.subsystems.Intake.Roller;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class RollerIONeo implements RollerIO {

    private CANSparkMax runExternal;
    private CANSparkMax runInternal; 

    public RollerIONeo(int runExternalID, int runInternalID) {
        runExternal = new CANSparkMax(runExternalID, MotorType.kBrushless);
        runInternal = new CANSparkMax(runInternalID, MotorType.kBrushless);

        runExternal.restoreFactoryDefaults();
        runExternal.setInverted(true);
        runExternal.burnFlash();

        runInternal.restoreFactoryDefaults();
        runInternal.setInverted(false);
        runInternal.burnFlash();
    }

    @Override
    public void setSpeed(double speed) {
        runExternal.set(speed);
        runInternal.set(speed);
    }

    @Override
    public void updateInputs(RollerIOInputs inputs) {
        
    }
}
