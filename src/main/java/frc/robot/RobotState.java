package frc.robot;

import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Drivetrain.Drivetrain;
import frc.robot.subsystems.Drivetrain.Drivetrain.DriveState;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.Intake.IntakeState;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.Shooter.ShooterState;
import org.littletonrobotics.junction.Logger;

public class RobotState {
  private Shooter shooter;
  private Intake intake;
  private Drivetrain drive;

  public enum State {
    AMP,
    AMP_REVVING,
    SUBWOOFER,
    SUBWOOFER_REVVING,
    AUTO_AIM,
    AUTO_AIM_REVVING,
    PASSING,
    VOMITING,
    INTAKING,
    NOTE_HELD,
    IDLE
  }

  // private final LoggedDashboardChooser<State> stateChooser = new LoggedDashboardChooser<>("Robot
  // State");

  public RobotState(Shooter shooter, Climber climber, Intake intake, Drivetrain drive) {
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
    this.drive = drive;
  }

  public State currentState = State.IDLE;

  public void setState(State state) {
    currentState = state;
  }

  public void updateState() {
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

  public void setDriveState(DriveState newState) {
    drive.setState(newState);
  }

  public void onAutonomousInit() {
    drive.setState(DriveState.FOLLOW_PATH);
  }

  public void onTeleopInit() {
    drive.setState(DriveState.MANUAL);
    setState(State.IDLE);
  }
}
