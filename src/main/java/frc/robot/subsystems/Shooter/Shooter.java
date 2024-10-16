package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Limelight.Limelight;
import frc.robot.subsystems.Shooter.Feeder.FeederIO;
import frc.robot.subsystems.Shooter.Feeder.FeederIOInputsAutoLogged;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelIO;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelIOInputsAutoLogged;
import frc.robot.subsystems.Shooter.Pivot.PivotIO;
import frc.robot.subsystems.Shooter.Pivot.PivotIOInputsAutoLogged;
import frc.robot.utils.ShooterUtils;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class Shooter extends SubsystemBase {

  private FlywheelIO flywheelIO;
  private PivotIO pivotIO;
  private FeederIO feederIO;
  private Limelight limelight;

  private FlywheelIOInputsAutoLogged flywheelIOInputs = new FlywheelIOInputsAutoLogged();
  private PivotIOInputsAutoLogged pivotIOInputs = new PivotIOInputsAutoLogged();
  private FeederIOInputsAutoLogged feederIOInputs = new FeederIOInputsAutoLogged();

  // For tuning the shooter. Only takes effect if in TUNING state.
  private LoggedDashboardNumber flywheelLeftSpeed =
      new LoggedDashboardNumber("Shooter/Tuning/FlywheelLeftSpeed");
  private LoggedDashboardNumber flywheelRightSpeed =
      new LoggedDashboardNumber("Shooter/Tuning/FlywheelRightSpeed");
  private LoggedDashboardNumber pivotAngle = new LoggedDashboardNumber("Shooter/Tuning/PivotAngle");
  private LoggedDashboardNumber rollerVoltage =
      new LoggedDashboardNumber("Shooter/Tuning/RollerVoltage");

  public enum ShooterState {
    AMP,
    AMP_REVVING,
    SUBWOOFER,
    SUBWOOFER_REVVING,
    AUTO_AIM,
    AUTO_AIM_REVVING,
    PASSING,
    VOMITING,
    TUNING,
    INTAKING,
    IDLE
  }

  private ShooterState currentState = ShooterState.IDLE;

  public Shooter(FlywheelIO flywheelIO, PivotIO pivotIO, FeederIO feederIO, Limelight limelight) {
    this.flywheelIO = flywheelIO;
    this.pivotIO = pivotIO;
    this.feederIO = feederIO;
    this.limelight = limelight;
  }

  public void setState(ShooterState to) {
    currentState = to;
  }

  public ShooterState getState() {
    return currentState;
  }

  public double getAngleFromDistance() {
    return ShooterUtils.instance.getAngleFromDistance(limelight.getDistance());
  }

  public double getSpeedFromDistance() {
    return ShooterUtils.instance.getSpeedFromDistance(limelight.getDistance());
  }

  public double getFlywheelSpeed() {
    return flywheelIO.getSpeed();
  }

  public double getPivotAngle() {
    return pivotIO.getAngle();
  }

  @Override
  public void periodic() {

    flywheelIO.updateInputs(flywheelIOInputs);
    Logger.processInputs("Shooter/Flywheel", flywheelIOInputs);

    pivotIO.updateInputs(pivotIOInputs);
    Logger.processInputs("Shooter/Pivot", pivotIOInputs);

    feederIO.updateInputs(feederIOInputs);
    Logger.processInputs("Shooter/Feeder", feederIOInputs);

    Logger.recordOutput("Shooter/State", currentState.toString());

    switch (currentState) {
      case AMP:
        flywheelIO.setSpeed(ShooterConstants.AMP_SPEED.get(), ShooterConstants.AMP_SPEED.get());
        pivotIO.setAngle(ShooterConstants.AMP_ANGLE.get());
        feederIO.setVoltage(ShooterConstants.AMP_ROLLER_VOLTAGE.get());
        break;
      case AMP_REVVING:
        flywheelIO.setSpeed(ShooterConstants.AMP_SPEED.get(), ShooterConstants.AMP_SPEED.get());
        pivotIO.setAngle(ShooterConstants.AMP_ANGLE.get());
        feederIO.setVoltage(0);
        break;
      case SUBWOOFER:
        flywheelIO.setSpeed(
            ShooterConstants.SPEAKER_SPEED.get() * 0.35, ShooterConstants.SPEAKER_SPEED.get());
        pivotIO.setAngle(ShooterConstants.SPEAKER_ANGLE.get());
        feederIO.setVoltage(ShooterConstants.SPEAKER_ROLLER_VOLTAGE.get());
        break;
      case SUBWOOFER_REVVING:
        flywheelIO.setSpeed(
            ShooterConstants.SPEAKER_SPEED.get() * 0.35, ShooterConstants.SPEAKER_SPEED.get());
        feederIO.setVoltage(0);
        pivotIO.setAngle(ShooterConstants.SPEAKER_ANGLE.get());
        break;
      case AUTO_AIM_REVVING:
        flywheelIO.setSpeed(getSpeedFromDistance() * 0.35, getSpeedFromDistance());
        feederIO.setVoltage(0);
        pivotIO.setAngle(getAngleFromDistance());
        break;
      case AUTO_AIM:
        flywheelIO.setSpeed(getSpeedFromDistance() * 0.35, getSpeedFromDistance());
        feederIO.setVoltage(ShooterConstants.SPEAKER_ROLLER_VOLTAGE.get());
        pivotIO.setAngle(getAngleFromDistance());
        break;
      case INTAKING:
        feederIO.setVoltage(ShooterConstants.INTAKING_ROLLER_VOLTAGE.get());
        break;
      case PASSING:
        // TODO
        break;
      case VOMITING:
        // TODO
        break;
      case TUNING:
        flywheelIO.setSpeed(flywheelLeftSpeed.get(), flywheelRightSpeed.get());
        pivotIO.setAngle(pivotAngle.get());
        feederIO.setVoltage(rollerVoltage.get());
        break;
      case IDLE:
        flywheelIO.setSpeed(0, 0);
        feederIO.setVoltage(0);
        pivotIO.setAngle(Constants.ShooterConstants.SPEAKER_ANGLE.get());
        break;
      default:
        break;
    }
    flywheelIO.updatePIDControllers();
    pivotIO.updatePIDControllers();
  }
}
