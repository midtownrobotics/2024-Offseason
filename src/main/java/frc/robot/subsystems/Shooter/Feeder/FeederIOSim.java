package frc.robot.subsystems.Shooter.Feeder;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;

public class FeederIOSim implements FeederIO {
    private final DCMotorSim rollerTopSim = new DCMotorSim(
    DCMotor.getNEO(1),
    Constants.ShooterConstants.Simulation.FEEDER_GEARING.get(),
    Constants.ShooterConstants.Simulation.FEEDER_MOI.get());
 
    private final DCMotorSim rollerBottomSim = new DCMotorSim(
    DCMotor.getNEO(1),
    Constants.ShooterConstants.Simulation.FEEDER_GEARING.get(), 
    Constants.ShooterConstants.Simulation.FEEDER_GEARING.get());

    @Override
    public void setVoltage(double voltage) {
       rollerTopSim.setInputVoltage(voltage); 
       rollerBottomSim.setInputVoltage(voltage);
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        rollerTopSim.update(0.02);
        rollerBottomSim.update(0.02);

        inputs.topOutputVoltage = MathUtil.clamp(rollerTopSim.getOutput(0), -12.0, 12.0);
        inputs.bottomOutputVoltage = MathUtil.clamp(rollerBottomSim.getOutput(0), -12.0, 12.0);

        inputs.topIsOn = rollerTopSim.getAngularVelocityRPM() > 0.01;
        inputs.bottomIsOn = rollerBottomSim.getAngularVelocityRPM() > 0.01;

        inputs.topVelocityRPM = rollerTopSim.getAngularVelocityRPM();
        inputs.bottomVelocityRPM = rollerBottomSim.getAngularVelocityRPM();

        inputs.bottomTempFahrenheit = 0.0; 
        inputs.topTempFahrenheit = 0.0;
        
        inputs.topCurrentAmps = rollerTopSim.getCurrentDrawAmps();
        inputs.bottomCurrentAmps = rollerBottomSim.getCurrentDrawAmps(); 
    }
}