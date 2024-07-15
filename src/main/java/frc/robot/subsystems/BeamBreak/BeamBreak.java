package frc.robot.subsystems.BeamBreak;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.Constants.IntakeConstants;
import frc.robot.RobotState.State;
import frc.robot.subsystems.BeamBreak.BeamBreakIO.BeamBreakIO;
import frc.robot.subsystems.BeamBreak.BeamBreakIO.BeamBreakIOInputsAutoLogged;

public class BeamBreak extends SubsystemBase{
    private RobotState robotState;

    private BeamBreakIO beamBreakIO;
    private BeamBreakIOInputsAutoLogged beamBreakIOInputs = new BeamBreakIOInputsAutoLogged();

    private XboxController driver;
    private XboxController operator;

    private int beamBreakBrokenTime;
    private BeamBreakState currentState = BeamBreakState.IDLE;

    public enum BeamBreakState {
        IDLE,
        NOTE_HELD,
        COUNTING
    }

    public BeamBreak(BeamBreakIO beamBreakIO, RobotState robotState, int driverPort, int operatorPort) {
        this.beamBreakIO = beamBreakIO;
        this.robotState = robotState;
        this.driver = new XboxController(driverPort);
        this.operator = new XboxController(operatorPort);
    }

    @Override
    public void periodic() {

        if (!beamBreakIO.getIsBroken() && robotState.currentState != State.NOTE_HELD) {
            beamBreakBrokenTime = 0;
        } else {
            driver.setRumble(RumbleType.kBothRumble, 1);
            operator.setRumble(RumbleType.kBothRumble, 1);
            if (beamBreakBrokenTime >= IntakeConstants.BEAMBREAK_DELAY.get()) {
                robotState.currentState = State.NOTE_HELD;
            }
            if (beamBreakBrokenTime >= IntakeConstants.CONTROLLER_RUMBLE_TIME.get()) {
                driver.setRumble(RumbleType.kBothRumble, 0);
                operator.setRumble(RumbleType.kBothRumble, 0);
            }
            beamBreakBrokenTime++;
        }
        Logger.recordOutput("BeamBreak/State", currentState.toString());
        Logger.recordOutput("BeamBreak/Counter", beamBreakBrokenTime);
        beamBreakIO.updateInputs(beamBreakIOInputs);
        Logger.processInputs("BeamBreak/Inputs", beamBreakIOInputs);
    }
}
