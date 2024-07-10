package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import com.google.flatbuffers.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake.BeamBreakIO.BeamBreakIO;
import frc.robot.subsystems.Intake.BeamBreakIO.BeamBreakIOInputsAutoLogged;
import frc.robot.subsystems.Intake.Roller.RollerIO;
import frc.robot.subsystems.Intake.Roller.RollerIOInputsAutoLogged;

public class Intake extends SubsystemBase{

    private RollerIO rollerIO;
    private BeamBreakIO beamBreakIO;

    private RollerIOInputsAutoLogged rollerIOInputs = new RollerIOInputsAutoLogged();
    private BeamBreakIOInputsAutoLogged beamBreakIOInputs = new BeamBreakIOInputsAutoLogged();
    private double counter = 0;
    private RobotState robotState;

    public enum IntakeState {
        INTAKING,
        VOMITING,
        TUNING,
        IDLE,
        SHOOTING
    }

    private IntakeState currentSetState = IntakeState.IDLE;

    public Intake(RollerIO rollerIO, BeamBreakIO beamBreakIO, RobotState robotState) {
        this.rollerIO = rollerIO;
        this.beamBreakIO = beamBreakIO;
        this.robotState = robotState;

    }

    public void setState(IntakeState to) {
        currentSetState = to;
    }
 
    @Override
    public void periodic() {

        rollerIO.updateInputs(rollerIOInputs);
        Logger.processInputs("Intake/Roller", rollerIOInputs);

        beamBreakIO.updateInputs(beamBreakIOInputs);
        Logger.processInputs("Intake/BeamBreak", beamBreakIOInputs);

        Logger.recordOutput("Intake/State", currentSetState.toString());

        switch (currentSetState) {
            case INTAKING:
                if (!beamBreakIO.getIsBroken()) {
                    rollerIO.setSpeed(1);
                    counter = 0;
                } else if (beamBreakIO.getIsBroken() && counter >= IntakeConstants.BEAMBREAK_DELAY.get()) {
                    robotState.currentState = RobotState.State.NOTE_HELD;
                } else {
                    rollerIO.setSpeed(1);
                    counter++;
                }
                break;
            case SHOOTING:
                rollerIO.setSpeed(1);
                break;
            case VOMITING:
                rollerIO.setSpeed(-1);
                break;
            case TUNING:
                // TODO
                break;
            case IDLE:
                rollerIO.setSpeed(0);
                break;
            default:
                break;
        }
    }
}
