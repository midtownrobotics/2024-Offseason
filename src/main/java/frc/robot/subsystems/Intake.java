package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    public enum IntakeState {
        INTAKING,
        VOMITING,
        TUNING,
        IDLE
    }

    private IntakeState currentState = IntakeState.IDLE;

    public void changeState(IntakeState to) {
        currentState = to;
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Intake/State", currentState.toString());

        switch (currentState) {
            case INTAKING:
                
                break;
            case VOMITING:
                
                break;
            case TUNING:
                
                break;
            case IDLE:

                break;
            default:
                break;
        }
    }
}
