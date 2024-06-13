package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Shooter.ShooterState;

public class RobotState extends SubsystemBase {
    private Shooter shooter;
    private Climber climber;
    private Intake intake;

    public RobotState(Shooter shooter, Climber climber, Intake intake) {
        this.shooter = shooter;
        this.climber = climber;
        this.intake = intake;
    }

    public enum State {
        AMP,
        SUBWOOFER,
        REVVING,
        AUTO_AIM,
        PASSING,
        VOMITING,
        INTAKING,
        IDLE
    }

    private State currentState = State.IDLE;

    @Override
    public void periodic() {
        switch (currentState) {
            case AMP:
                shooter.changeState(ShooterState.AMP);
                intake.changeState(IntakeState.IDLE);
                break;
            case SUBWOOFER:
                shooter.changeState(ShooterState.SUBWOOFER);
                intake.changeState(IntakeState.IDLE);
                break;
            case REVVING:
                shooter.changeState(ShooterState.REVVING);
                intake.changeState(IntakeState.IDLE);
                break;
            case AUTO_AIM:
                shooter.changeState(ShooterState.AUTO_AIM);
                intake.changeState(IntakeState.IDLE);
                break;
            case PASSING:
                shooter.changeState(ShooterState.PASSING);
                intake.changeState(IntakeState.IDLE);
                break;
            case VOMITING:
                shooter.changeState(ShooterState.VOMITING);
                intake.changeState(IntakeState.VOMITING);
                break;
            case INTAKING:
                shooter.changeState(ShooterState.IDLE);
                intake.changeState(IntakeState.INTAKING);
                break;
            case IDLE:
                shooter.changeState(ShooterState.IDLE);
                intake.changeState(IntakeState.IDLE);
                break;
            default:
                break;
        }
    }
}
