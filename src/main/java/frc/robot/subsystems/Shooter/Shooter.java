package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter.Feeder.FeederIO;
import frc.robot.subsystems.Shooter.Feeder.FeederIOInputsAutoLogged;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelIO;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelIOInputsAutoLogged;
import frc.robot.subsystems.Shooter.Pivot.PivotIO;
import frc.robot.subsystems.Shooter.Pivot.PivotIOInputsAutoLogged;

public class Shooter extends SubsystemBase {

    private FlywheelIO flywheelIO;
    private PivotIO pivotIO;
    private FeederIO feederIO;

    private FlywheelIOInputsAutoLogged flywheelIOInputs = new FlywheelIOInputsAutoLogged();
    private PivotIOInputsAutoLogged pivotIOInputs = new PivotIOInputsAutoLogged();
    private FeederIOInputsAutoLogged feederIOInputs = new FeederIOInputsAutoLogged();

    // For tuning the shooter. Only takes effect if in TUNING state.
    private LoggedDashboardNumber flywheelLeftSpeed = new LoggedDashboardNumber("Shooter/Tuning/FlywheelLeftSpeed");
    private LoggedDashboardNumber flywheelRightSpeed = new LoggedDashboardNumber("Shooter/Tuning/FlywheelRightSpeed");
    private LoggedDashboardNumber pivotAngle = new LoggedDashboardNumber("Shooter/Tuning/PivotAngle");
    private LoggedDashboardNumber rollerVoltage = new LoggedDashboardNumber("Shooter/Tuning/RollerVoltage");
    
    public enum ShooterState {
        AMP,
        AMP_REVVING,
        SUBWOOFER,
        SUBWOOFER_REVVING,
        AUTO_AIM,
        PASSING,
        VOMITING,
        TUNING,
        INTAKING,
        IDLE
    }

    private ShooterState currentState = ShooterState.IDLE;

    public Shooter(FlywheelIO flywheelIO, PivotIO pivotIO, FeederIO feederIO) {
        this.flywheelIO = flywheelIO;
        this.pivotIO = pivotIO;
        this.feederIO = feederIO;
    }

    public void setState(ShooterState to) {
        currentState = to;
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
                flywheelIO.setSpeed(ShooterConstants.SPEAKER_SPEED.get() * 0.35, ShooterConstants.SPEAKER_SPEED.get());
                pivotIO.setAngle(ShooterConstants.SPEAKER_ANGLE.get());
                feederIO.setVoltage(ShooterConstants.SPEAKER_ROLLER_VOLTAGE.get());
                break;
            case SUBWOOFER_REVVING:
                flywheelIO.setSpeed(ShooterConstants.SPEAKER_SPEED.get() * 0.35, ShooterConstants.SPEAKER_SPEED.get());
                feederIO.setVoltage(0);
                pivotIO.setAngle(ShooterConstants.SPEAKER_ANGLE.get());
                break;
            case INTAKING:
                feederIO.setVoltage(ShooterConstants.INTAKING_ROLLER_VOLTAGE.get());
                break;
            case AUTO_AIM:
                // TODO
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
                break;
            default:
                break;
        }
    }
}
