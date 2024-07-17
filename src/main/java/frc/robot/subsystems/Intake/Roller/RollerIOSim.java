package frc.robot.subsystems.Intake.Roller;

 import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class RollerIOSim implements RollerIO {
    // TODO: make sure we actually use 550s on this
    private final DCMotorSim simRunInternal = new DCMotorSim(
    DCMotor.getNeo550(1),
    Constants.IntakeConstants.GEARING, 
    Constants.IntakeConstants.MOI);

    private final DCMotorSim simRunExternal = new DCMotorSim(DCMotor.getNeo550(1),
    Constants.IntakeConstants.GEARING,
    Constants.IntakeConstants.MOI);
    
    
    @Override
    public void updateInputs(RollerIOInputs inputs) {
       if (DriverStation.isDisabled()) {
        simRunInternal.setInputVoltage(0);
        simRunExternal.setInputVoltage(0);
       }

       simRunInternal.update(0.02);
       simRunExternal.update(0.02);

       inputs.internalOutputVoltage = simRunInternal.getOutput(0);

    }

    @Override
    public void setSpeed(double speed) {
        simRunInternal.setInput(speed);
        simRunExternal.setInput(speed);
    }
}
