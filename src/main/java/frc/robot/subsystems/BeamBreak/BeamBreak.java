package frc.robot.subsystems.BeamBreak;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.RobotState;
import frc.robot.RobotState.State;
import frc.robot.subsystems.BeamBreak.BeamBreakIO.BeamBreakIO;
import frc.robot.subsystems.BeamBreak.BeamBreakIO.BeamBreakIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class BeamBreak extends SubsystemBase {
  private RobotState robotState;

  private BeamBreakIO beamBreakIO;
  private BeamBreakIOInputsAutoLogged beamBreakIOInputs = new BeamBreakIOInputsAutoLogged();

  private int beamBreakBrokenTime;
  private BeamBreakState currentState = BeamBreakState.IDLE;

  public enum BeamBreakState {
    IDLE,
    NOTE_HELD,
    COUNTING
  }

  public BeamBreak(
      BeamBreakIO beamBreakIO, RobotState robotState) {
    this.beamBreakIO = beamBreakIO;
    this.robotState = robotState;
  }

  @Override
  public void periodic() {

    if (beamBreakIO.getIsBroken()) {

      if (
        beamBreakBrokenTime == IntakeConstants.BEAMBREAK_DELAY.get() &&
        robotState != null &&
        robotState.currentState == State.INTAKING
      ){
        robotState.setState(State.NOTE_HELD);
        currentState = BeamBreakState.NOTE_HELD;
      } else {
        currentState = BeamBreakState.COUNTING;
      }

      beamBreakBrokenTime++;
    } else {
      currentState = BeamBreakState.IDLE;
      beamBreakBrokenTime = 0;
    }

    Logger.recordOutput("BeamBreak/State", currentState.toString());
    Logger.recordOutput("BeamBreak/Counter", beamBreakBrokenTime);

    beamBreakIO.updateInputs(beamBreakIOInputs);
    Logger.processInputs("BeamBreak/Inputs", beamBreakIOInputs);
  }

  public void setRobotState(RobotState robotState) {
    this.robotState = robotState;
  }
}
