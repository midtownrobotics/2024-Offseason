package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelIO;
import frc.robot.subsystems.Shooter.Pivot.PivotIO;
import frc.robot.subsystems.Shooter.Roller.RollerIO;

public class Shooter extends SubsystemBase {

    private FlywheelIO flywheelIO;
    private PivotIO pivotIO;
    private RollerIO rollerIO;

    private LoggedDashboardNumber flywheelLeftSpeed = new LoggedDashboardNumber("Shooter/FlywheelLeftSpeed");
    private LoggedDashboardNumber flywheelRightSpeed = new LoggedDashboardNumber("Shooter/FlywheelRightSpeed");
    private LoggedDashboardNumber pivotAngle = new LoggedDashboardNumber("Shooter/PivotAngle");
    private LoggedDashboardNumber rollerVoltage = new LoggedDashboardNumber("Shooter/RollerVoltage");
    private LoggedDashboardNumber rollerVoltageFromFlywheelSpeed = new LoggedDashboardNumber("Shooter/RollerVoltageFromFlywheelSpeed");

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

    public Shooter(FlywheelIO flywheelIO, PivotIO pivotIO, RollerIO rollerIO) {
        this.flywheelIO = flywheelIO;
        this.pivotIO = pivotIO;
        this.rollerIO = rollerIO;
    }

    public void setState(ShooterState to) {
        currentState = to;
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Shooter/State", currentState.toString());

        flywheelIO.setSpeed(0, 0);

        switch (currentState) {
            case AMP:
                flywheelIO.setSpeed(ShooterConstants.AMP_SPEED, ShooterConstants.AMP_SPEED);
                pivotIO.setAngle(ShooterConstants.AMP_ANGLE);
                rollerIO.setVoltage(getRollerVoltage(ShooterConstants.AMP_SPEED));
                break;
            case SUBWOOFER:
                flywheelIO.setSpeed(ShooterConstants.SPEAKER_SPEED * 0.35, ShooterConstants.SPEAKER_SPEED);
                pivotIO.setAngle(ShooterConstants.SPEAKER_ANGLE);
                rollerIO.setVoltage(getRollerVoltage(ShooterConstants.SPEAKER_SPEED));
                break;
            case REVVING:
                // todo
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
                if (rollerVoltage.get() == 0) {
                    rollerIO.setVoltage(getRollerVoltage(rollerVoltageFromFlywheelSpeed.get()));
                } else {
                    rollerIO.setVoltage(rollerVoltage.get());
                }
                break;
            case IDLE:
                flywheelIO.setSpeed(0, 0);
                rollerIO.setVoltage(getRollerVoltage(0));
                break;
            default:
                break;
        }
    }

    public double getRollerVoltage(double flywheelSpeed) {
        if (flywheelSpeed == 0) {
            return 0;
        }
        return Math.min(1, (Math.max((flywheelSpeed / 700.0), .3))) * 12;
    }
}
