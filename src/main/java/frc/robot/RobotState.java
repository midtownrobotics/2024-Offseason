package frc.robot;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.Shooter.ShooterState;

public class RobotState extends SubsystemBase {
    private Shooter shooter;
    private Climber climber;
    private Intake intake;

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

    private final LoggedDashboardChooser<State> stateChooser = new LoggedDashboardChooser<>("Robot State");

    public RobotState(Shooter shooter, Climber climber, Intake intake) {
        stateChooser.addOption("AMP", State.AMP);
        stateChooser.addOption("SUBWOOFER", State.SUBWOOFER);
        stateChooser.addOption("REVVING", State.REVVING);
        stateChooser.addOption("AUTO_AIM", State.AUTO_AIM);
        stateChooser.addOption("PASSING", State.PASSING);
        stateChooser.addOption("VOMITING", State.VOMITING);
        stateChooser.addOption("INTAKING", State.INTAKING);
        stateChooser.addOption("IDLE", State.IDLE);

        this.shooter = shooter;
        this.climber = climber;
        this.intake = intake;
    }

    private State currentState = State.IDLE;

    @Override
    public void periodic() {

        if (stateChooser.get() != null) {
            currentState = stateChooser.get();
        } else {
            currentState = State.IDLE;
        };

        Logger.recordOutput("Robot/State", currentState.toString());

        switch (currentState) {
            case AMP:
                shooter.setState(ShooterState.AMP);
                intake.setState(IntakeState.IDLE);
                break;
            case SUBWOOFER:
                shooter.setState(ShooterState.SUBWOOFER);
                intake.setState(IntakeState.IDLE);
                break;
            case REVVING:
                shooter.setState(ShooterState.REVVING);
                intake.setState(IntakeState.IDLE);
                break;
            case AUTO_AIM:
                shooter.setState(ShooterState.AUTO_AIM);
                intake.setState(IntakeState.IDLE);
                break;
            case PASSING:
                shooter.setState(ShooterState.PASSING);
                intake.setState(IntakeState.IDLE);
                break;
            case VOMITING:
                shooter.setState(ShooterState.VOMITING);
                intake.setState(IntakeState.VOMITING);
                break;
            case INTAKING:
                shooter.setState(ShooterState.IDLE);
                intake.setState(IntakeState.INTAKING);
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
