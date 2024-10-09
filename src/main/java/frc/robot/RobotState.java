package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.Intake.IntakeState;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.Shooter.ShooterState;

public class RobotState extends SubsystemBase {
    private Shooter shooter;
    private Intake intake;

    public enum State {
        AMP,
        AMP_REVVING,
        SUBWOOFER,
        SUBWOOFER_REVVING,
        AUTO_AIM,
        AUTO_AIM_REVVING,
        PASSING,
        SUBWOOFER_REVSHOOT,
        VOMITING,
        INTAKING,
        NOTE_HELD,
        IDLE
    }

    // private final LoggedDashboardChooser<State> stateChooser = new LoggedDashboardChooser<>("Robot State");

    public RobotState(Shooter shooter, Climber climber, Intake intake) {
        // stateChooser.addOption("AMP", State.AMP);
        // stateChooser.addOption("AMP_REVVING", State.AMP_REVVING);
        // stateChooser.addOption("SUBWOOFER", State.SUBWOOFER);
        // stateChooser.addOption("SUBWOOFER_REVVING", State.SUBWOOFER_REVVING);
        // stateChooser.addOption("AUTO_AIM", State.AUTO_AIM);
        // stateChooser.addOption("PASSING", State.PASSING);
        // stateChooser.addOption("VOMITING", State.VOMITING);
        // stateChooser.addOption("INTAKING", State.INTAKING);
        // stateChooser.addOption("IDLE", State.IDLE);

        this.shooter = shooter;
        this.intake = intake;
    }

    public State currentState = State.IDLE;

    public void setState(State state) {
        currentState = state;
    }

    @Override
    public void periodic() {

        // if (stateChooser.get() != null) {
        //     currentState = stateChooser.get();
        // } else {
        //     currentState = State.IDLE;
        // };
        
        Logger.recordOutput("Robot/State", currentState.toString());

        switch (currentState) {
            case AMP:
                shooter.setState(ShooterState.AMP);
                intake.setState(IntakeState.SHOOTING);
                break;
            case AMP_REVVING:
                shooter.setState(ShooterState.AMP_REVVING);
                intake.setState(IntakeState.IDLE);
                break;
            case SUBWOOFER:
                shooter.setState(ShooterState.SUBWOOFER);
                intake.setState(IntakeState.SHOOTING);
                break;
            case SUBWOOFER_REVVING:
                shooter.setState(ShooterState.SUBWOOFER_REVVING);
                intake.setState(IntakeState.IDLE);
                break;
            case AUTO_AIM:
                shooter.setState(ShooterState.AUTO_AIM);
                intake.setState(IntakeState.SHOOTING);
                break;
            case AUTO_AIM_REVVING:
                shooter.setState(ShooterState.AUTO_AIM_REVVING);
                intake.setState(IntakeState.IDLE);
                break;
            case PASSING:
                shooter.setState(ShooterState.PASSING);
                intake.setState(IntakeState.IDLE);
                break;
            case SUBWOOFER_REVSHOOT:
                shooter.setState(ShooterState.SUBWOOFER_REVVING);
                intake.setState(IntakeState.IDLE);
                if (shooter.getFlywheelSpeed() >= ShooterConstants.SPEAKER_SPEED.get()) {
                    shooter.setState(ShooterState.SUBWOOFER);
                }
                break;
            case VOMITING:
                shooter.setState(ShooterState.VOMITING);
                intake.setState(IntakeState.VOMITING);
                break;
            case INTAKING:
                shooter.setState(ShooterState.INTAKING);
                intake.setState(IntakeState.INTAKING);
                break;
            case NOTE_HELD:
                shooter.setState(ShooterState.IDLE);
                intake.setState(IntakeState.IDLE);
                break;
            case IDLE:
                shooter.setState(ShooterState.IDLE);
                intake.setState(IntakeState.IDLE);
                break;
            default:
                break;
        }
    }
}
