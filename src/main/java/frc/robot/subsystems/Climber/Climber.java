package frc.robot.subsystems.Climber;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber.Winch.WinchIO;

public class Climber extends SubsystemBase{
    private ClimberState currentState = ClimberState.IDLE;
    private WinchIO winchIO;

    private LoggedDashboardNumber winchLeftSpeed = new LoggedDashboardNumber("Climber/WinchLeftSpeed");
    private LoggedDashboardNumber winchRightSpeed = new LoggedDashboardNumber("Climber/WinchRightSpeed");
    
    
    public enum ClimberState {
        EXTENDING,
        RETRACTING,
        TUNING,
        IDLE
    }

    private final LoggedDashboardChooser<ClimberState> stateChooser = new LoggedDashboardChooser<>("Climber");

    public Climber(WinchIO winchIO) {
        this.winchIO = winchIO;

        stateChooser.addOption("EXTENDING", ClimberState.EXTENDING);
        stateChooser.addOption("RETRACTING", ClimberState.RETRACTING);
        stateChooser.addOption("TUNING", ClimberState.TUNING);
        stateChooser.addOption("IDLE", ClimberState.IDLE);
    }

    public void setState(ClimberState desiredState) {
        currentState = desiredState;
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Climber/State", currentState.toString());

        if (stateChooser.get() != null) {
            currentState = stateChooser.get();
        } else {
            currentState = ClimberState.IDLE;
        };

        switch (currentState) {
            case EXTENDING:
                winchIO.setSpeed(ClimberConstants.LEFT_WINCH_SPEED, ClimberConstants.RIGHT_WINCH_SPEED);
                break;
            case RETRACTING: 
                winchIO.setSpeed(-ClimberConstants.LEFT_WINCH_SPEED, -ClimberConstants.RIGHT_WINCH_SPEED);
                break;
            case TUNING: 
                winchIO.setSpeed(winchLeftSpeed.get(), winchRightSpeed.get());
                break;
            case IDLE:
                winchIO.setSpeed(0, 0);
                break;
            default:
                break;
        }
    }
}
