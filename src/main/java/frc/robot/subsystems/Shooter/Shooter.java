package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter.Feeder.FeederIO;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelIO;
import frc.robot.subsystems.Shooter.Pivot.PivotIO;

public class Shooter extends SubsystemBase {

    private FlywheelIO flywheelIO;
    private PivotIO pivotIO;
    private FeederIO feederIO;

    // For tuning the shooter. Only takes effect if in TUNING state.
    private LoggedDashboardNumber flywheelLeftSpeed = new LoggedDashboardNumber("Shooter/Tuning/FlywheelLeftSpeed");
    private LoggedDashboardNumber flywheelRightSpeed = new LoggedDashboardNumber("Shooter/Tuning/FlywheelRightSpeed");
    private LoggedDashboardNumber pivotAngle = new LoggedDashboardNumber("Shooter/Tuning/PivotAngle");
    private LoggedDashboardNumber rollerVoltage = new LoggedDashboardNumber("Shooter/Tuning/RollerVoltage");

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
        Logger.recordOutput("Shooter/State", currentState.toString());

        switch (currentState) {
            case AMP:
                flywheelIO.setSpeed(ShooterConstants.AMP_SPEED.get(), ShooterConstants.AMP_SPEED.get());
                pivotIO.setAngle(ShooterConstants.AMP_ANGLE.get());
                feederIO.setVoltage(ShooterConstants.AMP_ROLLER_VOLTAGE.get());
                break;
            case SUBWOOFER:
                flywheelIO.setSpeed(ShooterConstants.SPEAKER_SPEED.get() * 0.35, ShooterConstants.SPEAKER_SPEED.get());
                pivotIO.setAngle(ShooterConstants.SPEAKER_ANGLE.get());
                feederIO.setVoltage(ShooterConstants.SPEAKER_ROLLER_VOLTAGE.get());
                break;
            case REVVING:
                flywheelIO.setSpeed(ShooterConstants.SPEAKER_SPEED.get() * 0.35, ShooterConstants.SPEAKER_SPEED.get());
                feederIO.setVoltage(0);
                pivotIO.setAngle(ShooterConstants.SPEAKER_ANGLE.get());
                break;
            case AUTO_AIM:
                // todo
                break;
            case PASSING:
                // todo
                break;
            case VOMITING:
                // todo
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
