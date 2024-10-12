package frc.robot.subsystems.BeamBreak;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
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

    private double brokenTime;
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
        double time = Timer.getFPGATimestamp();

        if (beamBreakIO.getIsBroken()) {
            if (time - brokenTime >= IntakeConstants.BEAMBREAK_DELAY.get()) {
                robotState.currentState = State.NOTE_HELD;
            }

            if (time - brokenTime < IntakeConstants.CONTROLLER_RUMBLE_TIME.get()) {
                driver.setRumble(RumbleType.kBothRumble, 1);
                operator.setRumble(RumbleType.kBothRumble, 1);
            } else {    
                driver.setRumble(RumbleType.kBothRumble, 0);
                operator.setRumble(RumbleType.kBothRumble, 0);
            }
        } else {
            brokenTime = time;
        }

        Logger.recordOutput("BeamBreak/State", currentState.toString());
        Logger.recordOutput("BeamBreak/Counter", time - brokenTime);

        beamBreakIO.updateInputs(beamBreakIOInputs);
        Logger.processInputs("BeamBreak/Inputs", beamBreakIOInputs);
    }
}
