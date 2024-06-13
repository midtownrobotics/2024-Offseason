package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    public enum ShooterState {
        AMP,
        SUBWOOFER,
        REVVING,
        AUTO_AIM,
        PASSING,
        VOMITING,
        TUNING,
        IDLE
    }

    private ShooterState currentState = ShooterState.IDLE;

    public void changeState(ShooterState to) {
        currentState = to;
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Shooter/State", currentState.toString());

        switch (currentState) {
            case AMP:
                
                break;
            case SUBWOOFER:
                
                break;
            case REVVING:
                
                break;
            case AUTO_AIM:
                
                break;
            case PASSING:
                
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
