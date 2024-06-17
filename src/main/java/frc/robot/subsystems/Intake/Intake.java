package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake.Roller.RollerIO;

public class Intake extends SubsystemBase{

    private RollerIO rollerIO;

    public enum IntakeState {
        INTAKING,
        VOMITING,
        TUNING,
        IDLE
    }

    private boolean beamBreakTripped = false; // TODO: Actually add beam break code

    private IntakeState currentSetState = IntakeState.IDLE;

    public Intake(RollerIO rollerIO) {
        this.rollerIO = rollerIO;
    }

    public void changeState(IntakeState to) {
        currentSetState = to;
    }

    @Override
    public void periodic() {

        IntakeState state = currentSetState;

        if (beamBreakTripped == true) {
            state = IntakeState.IDLE;
        }

        Logger.recordOutput("Intake/State", state.toString());

        switch (state) {
            case INTAKING:
                rollerIO.setSpeed(1);
                break;
            case VOMITING:
                rollerIO.setSpeed(-1);
                break;
            case TUNING:
                
                break;
            case IDLE:
                rollerIO.setSpeed(0);
                break;
            default:
                break;
        }
    }
}
