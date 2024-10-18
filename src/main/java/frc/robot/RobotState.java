package frc.robot;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Drivetrain.Drivetrain;
import frc.robot.subsystems.Drivetrain.Drivetrain.DriveState;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.Intake.IntakeState;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.Shooter.ShooterState;

public class RobotState {
  public Shooter shooter;
  public Intake intake;
  public Drivetrain drive;

  public enum State {
    AMP,
    AMP_REVVING,
    AMP_REVSHOOT,
    SUBWOOFER,
    SUBWOOFER_REVVING,
    SUBWOOFER_REVSHOOT,
    AUTO_AIM,
    AUTO_AIM_REVVING,
    INTAKE_REVVING,
    AUTO_AIM_REVSHOOT,
    PASSING,
    VOMITING,
    INTAKING,
    NOTE_HELD,
    TELEOP,
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
      case AMP_REVSHOOT:
        shooter.setState(ShooterState.AMP_REVVING);
        intake.setState(IntakeState.IDLE);
        if (shooter.getFlywheelSpeed() >= (ShooterConstants.AMP_SPEED.get())) {
            shooter.setState(ShooterState.AMP);
            intake.setState(IntakeState.SHOOTING);
        }
        break;
      case SUBWOOFER:
        shooter.setState(ShooterState.SUBWOOFER);
        intake.setState(IntakeState.SHOOTING);
        break;
      case SUBWOOFER_REVVING:
        shooter.setState(ShooterState.SUBWOOFER_REVVING);
        intake.setState(IntakeState.IDLE);
        break;
      case INTAKE_REVVING:
        shooter.setState(ShooterState.SUBWOOFER_REVVING);
        intake.setState(IntakeState.INTAKING);
        break;
      case SUBWOOFER_REVSHOOT:
        shooter.setState(ShooterState.SUBWOOFER_REVVING);
        intake.setState(IntakeState.IDLE);
        if (shooter.getFlywheelSpeed() >= (ShooterConstants.SPEAKER_SPEED.get())) {
           setState(State.SUBWOOFER);
        }
        break;
      case AUTO_AIM:
        shooter.setState(ShooterState.AUTO_AIM);
        intake.setState(IntakeState.SHOOTING);
        break;
      case AUTO_AIM_REVVING:
        shooter.setState(ShooterState.AUTO_AIM_REVVING);
        intake.setState(IntakeState.IDLE);
        break;
      case AUTO_AIM_REVSHOOT:
        shooter.setState(ShooterState.AUTO_AIM_REVVING);
        intake.setState(IntakeState.IDLE);
        if (
          shooter.getFlywheelSpeed() >= (shooter.getSpeedFromDistance() - ShooterConstants.SPEAKER_SPEED_TOLERANCE.get()) &&
          (
            shooter.getPivotAngle() >= (shooter.getAngleFromDistance() - ShooterConstants.PIVOT_ANGLE_TOLERANCE.get()) &&
            shooter.getPivotAngle() <= (shooter.getAngleFromDistance() + ShooterConstants.PIVOT_ANGLE_TOLERANCE.get())
          )
        ) {
            setState(State.AUTO_AIM);
        }
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
      case TELEOP:
        break;
      default:
        break;
    }
  }

  public ShooterState getShooterState() {
    return shooter.getState();
  }

  public void setDriveState(DriveState newState) {
    drive.setState(newState);
  }

  public void onAutonomousInit() {
    drive.setState(DriveState.FOLLOW_PATH);
    setState(State.IDLE);
  }

  public void onTeleopInit() {
    drive.setState(DriveState.MANUAL);
    setState(State.IDLE);
  }
}