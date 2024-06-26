package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake.BeamBreakIO.BeamBreakIO;
import frc.robot.subsystems.Intake.BeamBreakIO.BeamBreakIOInputsAutoLogged;
import frc.robot.subsystems.Intake.Roller.RollerIO;
import frc.robot.subsystems.Intake.Roller.RollerIOInputsAutoLogged;

public class Intake extends SubsystemBase{

    private RollerIO rollerIO;
    private BeamBreakIO beamBreakIIO;

    private RollerIOInputsAutoLogged rollerIOInputs = new RollerIOInputsAutoLogged();
    private BeamBreakIOInputsAutoLogged beamBreakIOInputs = new BeamBreakIOInputsAutoLogged();

    public enum IntakeState {
        INTAKING,
        VOMITING,
        TUNING,
        IDLE
    }

    private IntakeState currentSetState = IntakeState.IDLE;

    public Intake(RollerIO rollerIO, BeamBreakIO beamBreakIIO) {
        this.rollerIO = rollerIO;
        this.beamBreakIIO = beamBreakIIO;
    }

    public void setState(IntakeState to) {
        currentSetState = to;
    }
 
    @Override
    public void periodic() {

        rollerIO.updateInputs(rollerIOInputs);
        Logger.processInputs("Intake/Roller", rollerIOInputs);

        beamBreakIIO.updateInputs(beamBreakIOInputs);
        Logger.processInputs("Intake/BeamBreak", beamBreakIOInputs);

        Logger.recordOutput("Intake/State", currentSetState.toString());

        switch (currentSetState) {
            case INTAKING:
                if (!beamBreakIIO.getIsBroken()) {
                    rollerIO.setSpeed(1);
                } else {
                    rollerIO.setSpeed(0);
                }
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
