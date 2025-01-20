package frc.robot;

import frc.robot.subsystems.Drivetrain.Drivetrain;
import frc.robot.subsystems.Drivetrain.Drivetrain.DriveState;
import frc.robot.utils.IOProtectionXboxController;
import org.littletonrobotics.junction.Logger;

public class RobotState {
  public IOProtectionXboxController driver;
  public IOProtectionXboxController operator;
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
    TUNING,
    IDLE
  }

  // private final LoggedDashboardChooser<State> stateChooser = new LoggedDashboardChooser<>("Robot
  // State");

  public RobotState(Drivetrain drive, int driverPort, int operatorPort) {

    // stateChooser.addOption("AMP", State.AMP);
    // stateChooser.addOption("AMP_REVVING", State.AMP_REVVING);
    // stateChooser.addOption("SUBWOOFER", State.SUBWOOFER);
    // stateChooser.addOption("SUBWOOFER_REVVING", State.SUBWOOFER_REVVING);
    // stateChooser.addOption("AUTO_AIM", State.AUTO_AIM);
    // stateChooser.addOption("PASSING", State.PASSING);
    // stateChooser.addOption("VOMITING", State.VOMITING);
    // stateChooser.addOption("INTAKING", State.INTAKING);
    // stateChooser.addOption("IDLE", State.IDLE);

    this.drive = drive;
    this.driver = new IOProtectionXboxController(driverPort);
    this.operator = new IOProtectionXboxController(operatorPort);
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
        break;
      case AMP_REVVING:
        break;
      case AMP_REVSHOOT:
        break;
      case SUBWOOFER:
        break;
      case SUBWOOFER_REVVING:
        break;
      case INTAKE_REVVING:
        break;
      case SUBWOOFER_REVSHOOT:
        break;
      case AUTO_AIM:
        break;
      case AUTO_AIM_REVVING:
        break;
      case AUTO_AIM_REVSHOOT:
        break;
      case PASSING:
        break;
      case VOMITING:
        break;
      case INTAKING:
        break;
      case NOTE_HELD:
        break;
      case IDLE:
        break;
      case TELEOP:
        break;
      case TUNING:
        drive.setState(DriveState.TUNING);
      default:
        break;
    }

    setControllerRumble();
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
    setState(State.TELEOP);
  }

  public void setControllerRumble() {}
}
