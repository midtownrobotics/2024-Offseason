package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake.BeamBreakIO.BeamBreakIO;
import frc.robot.subsystems.Intake.Roller.RollerIO;

public class Intake extends SubsystemBase{

    private RollerIO rollerIO;
    private BeamBreakIO beamBreakIIO;

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

        Logger.recordOutput("Intake/State", currentSetState.toString());

        switch (currentSetState) {
            case INTAKING:
                if (beamBreakIIO.getIsBroken() == false) {
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
