package frc.robot;

import frc.robot.subsystems.BeamBreak.BeamBreak;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Drivetrain.Drivetrain;
import frc.robot.subsystems.Drivetrain.Drivetrain.DriveState;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.Intake.IntakeState;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.Shooter.ShooterState;
import frc.robot.utils.IOProtectionXboxController;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class RobotState {
  private Shooter shooter;
  private Intake intake;
  BeamBreak beamBreak;
  IOProtectionXboxController driver; 
  IOProtectionXboxController operator;

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

  public RobotState(Shooter shooter, 
                    Climber climber, 
                    Intake intake, 
                    Drivetrain drive, 
                    BeamBreak beamBreak, 
                    int driverPort, 
                    int operatorPort) {

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
    this.beamBreak = beamBreak;
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
        if (beamBreak.isBroken()) {
          setState(State.NOTE_HELD);
        }
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

    setControllerRumble(); 
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

  public void setControllerRumble(){
    if (beamBreak.isBroken() && 
        edu.wpi.first.wpilibj.RobotState.isTeleop() && 
        Constants.RUMBLE_DURATION > beamBreak.getBrokenTime()) {

      driver.setRumble(RumbleType.kBothRumble, 1);
      operator.setRumble(RumbleType.kBothRumble, 1);
    } else P
      driver.setRumble(RumbleType.kBothRumble, 0);
      operator.setRumble(RumbleType.kBothRumble, 0); 
    }
  }
}
