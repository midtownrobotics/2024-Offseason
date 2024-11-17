package frc.robot.subsystems.Intake.Roller;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;

public class RollerIOSim implements RollerIO {
    //TODO: Setup motor sim classes.
    private final DCMotorSim simRunInternal = new DCMotorSim(null, null, null);
    private final DCMotorSim simRunExternal = new DCMotorSim(null, null, null);
    
    
    @Override
    public void updateInputs(RollerIOInputs inputs) {
        simRunInternal.update(0.02);
        simRunExternal.update(0.02);

        inputs.internalOutputVoltage = MathUtil.clamp(simRunInternal.getOutput(0), -12.0, 12.0);
        inputs.externalOutputVoltage = MathUtil.clamp(simRunExternal.getOutput(0), -12.0, 12.0);

        inputs.internalIsOn = simRunInternal.getAngularVelocityRPM() > 0.01;
        inputs.externalIsOn = simRunExternal.getAngularVelocityRPM() > 0.01;

        inputs.internalVelocityRPM = simRunInternal.getAngularVelocityRPM();
        inputs.externalVelocityRPM = simRunExternal.getAngularVelocityRPM();

        inputs.externalTempFahrenheit = 0.0; 
        inputs.internalTempFahrenheit = 0.0;
        
        inputs.internalCurrentAmps = simRunInternal.getCurrentDrawAmps();
        inputs.externalCurrentAmps = simRunExternal.getCurrentDrawAmps();

    }

    @Override
    public void setSpeed(double speed) {
        simRunInternal.setInputVoltage(speed * Constants.THEORETICAL_RESTING_VOLTAGE);
        simRunExternal.setInputVoltage(speed * Constants.THEORETICAL_RESTING_VOLTAGE);
    }
}
