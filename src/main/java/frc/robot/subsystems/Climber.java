package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase{
    public enum ClimberState {
        EXTENDING,
        RETRACTING,
        TUNING,
        IDLE
    }

    private ClimberState currentState = ClimberState.IDLE;

    public void changeState(ClimberState to) {
        currentState = to;
    }

    @Override
    public void periodic() {
        switch (currentState) {
            case EXTENDING:
                
                break;
            case RETRACTING:
                
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
