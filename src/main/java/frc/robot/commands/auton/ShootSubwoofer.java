package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.RobotState.State;

public class ShootSubwoofer extends Command {
    private final RobotState robotState;
    private double startTime;

    public ShootSubwoofer(RobotState robotState) {
        this.robotState = robotState;
    }

    @Override
    public void initialize(){
        this.startTime = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isFinished() {
        double length = Constants.AutonConstants.AUTON_SHOOT_SUBWOOFER_LENGTH_SEC.get();

        if (Timer.getFPGATimestamp() - this.startTime >= length) {
            robotState.setState(State.IDLE);
            return true;
        }

        if (Timer.getFPGATimestamp() - this.startTime < length) {
            robotState.setState(State.SUBWOOFER);
        }

        return false;
    }

}
