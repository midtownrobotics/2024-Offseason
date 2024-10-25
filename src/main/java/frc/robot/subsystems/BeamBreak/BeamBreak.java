package frc.robot.subsystems.BeamBreak;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.BeamBreak.BeamBreakIO.BeamBreakIO;
import frc.robot.subsystems.BeamBreak.BeamBreakIO.BeamBreakIOInputsAutoLogged;
import frc.robot.subsystems.Intake.Intake.IntakeState;

import org.littletonrobotics.junction.Logger;

public class BeamBreak extends SubsystemBase {
  private RobotState robotState;

  private BeamBreakIO beamBreakIO;
  private BeamBreakIOInputsAutoLogged beamBreakIOInputs = new BeamBreakIOInputsAutoLogged();

  private XboxController driver;
  private XboxController operator;

  private int beamBreakBrokenTime;
  private BeamBreakState currentState = BeamBreakState.IDLE;

  private boolean isRumbling = false;

  public enum BeamBreakState {
    IDLE,
    NOTE_HELD,
    COUNTING
  }

  public BeamBreak(
      BeamBreakIO beamBreakIO, RobotState robotState, int driverPort, int operatorPort) {
    this.beamBreakIO = beamBreakIO;
    this.robotState = robotState;
    this.driver = new XboxController(driverPort);
    this.operator = new XboxController(operatorPort);
  }

  @Override
  public void periodic() {

    if (beamBreakIO.getIsBroken()) {
      if (beamBreakBrokenTime == 0) {
        if (edu.wpi.first.wpilibj.RobotState.isTeleop() && edu.wpi.first.wpilibj.RobotState.isEnabled()) {
          driver.setRumble(RumbleType.kBothRumble, 1);
          operator.setRumble(RumbleType.kBothRumble, 1);
          isRumbling = true;
        }
      }
      if (beamBreakBrokenTime == IntakeConstants.BEAMBREAK_DELAY.get()) {
        if (robotState != null && robotState.intake != null && robotState.intake.currentSetState == IntakeState.INTAKING) {
          robotState.intake.currentSetState = IntakeState.NOTE_HELD;
        }
      }
      if (beamBreakBrokenTime == IntakeConstants.CONTROLLER_RUMBLE_TIME.get()) {
        driver.setRumble(RumbleType.kBothRumble, 0);
        operator.setRumble(RumbleType.kBothRumble, 0);
        isRumbling = false;
      }
      beamBreakBrokenTime++;
    } else {
      beamBreakBrokenTime = 0;
    }

    Logger.recordOutput("BeamBreak/State", currentState.toString());
    Logger.recordOutput("BeamBreak/RumberOp", isRumbling);
    Logger.recordOutput("BeamBreak/Counter", beamBreakBrokenTime);

    beamBreakIO.updateInputs(beamBreakIOInputs);
    Logger.processInputs("BeamBreak/Inputs", beamBreakIOInputs);
  }

  public void setRobotState(RobotState robotState) {
    this.robotState = robotState;
  }
}
