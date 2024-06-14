package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelIO;
import frc.robot.subsystems.Shooter.Pivot.PivotIO;

public class Shooter extends SubsystemBase {

    private FlywheelIO flywheelIO;
    private PivotIO pivotIO;

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

    public Shooter(FlywheelIO flywheelIO, PivotIO pivotIO) {
        this.flywheelIO = flywheelIO;
        this.pivotIO = pivotIO;
    }

    public void changeState(ShooterState to) {
        currentState = to;
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Shooter/State", currentState.toString());

        flywheelIO.setVoltage(0, 0);

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
