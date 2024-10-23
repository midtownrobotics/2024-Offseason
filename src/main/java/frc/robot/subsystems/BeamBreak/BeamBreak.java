package frc.robot.subsystems.BeamBreak;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.BeamBreak.BeamBreakIO.BeamBreakIO;
import frc.robot.subsystems.BeamBreak.BeamBreakIO.BeamBreakIOInputsAutoLogged;

public class BeamBreak extends SubsystemBase {
  private BeamBreakIO beamBreakIO;
  private BeamBreakIOInputsAutoLogged beamBreakIOInputs = new BeamBreakIOInputsAutoLogged();
  private double initialBreakTimestamp;
  private boolean brokenLastCycle;

  public BeamBreak(BeamBreakIO beamBreakIO) {
    this.beamBreakIO = beamBreakIO;
  }

  public boolean isBroken() {
    return beamBreakIO.getIsBroken() && brokenLastCycle;
  }

  public double getBrokenTime() {
    return Timer.getFPGATimestamp() - initialBreakTimestamp;
  }

  @Override
  public void periodic() {
    if (beamBreakIO.getIsBroken() && !brokenLastCycle) {
      initialBreakTimestamp = Timer.getFPGATimestamp();
      brokenLastCycle = true;
    } else if (!beamBreakIO.getIsBroken()) {
      brokenLastCycle = false;
    }
    
    beamBreakIO.updateInputs(beamBreakIOInputs);
  }
}
