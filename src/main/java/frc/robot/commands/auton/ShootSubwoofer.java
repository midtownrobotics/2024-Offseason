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
        robotState.setState(State.SUBWOOFER_REVVING);
    }

    @Override
    public boolean isFinished() {
        double revLength = Constants.AutonConstants.AUTON_SHOOT_SUBWOOFER_REV_LENGTH_SEC.get();
        double length = Constants.AutonConstants.AUTON_SHOOT_SUBWOOFER_LENGTH_SEC.get() + revLength;

        if (Timer.getFPGATimestamp() - this.startTime >= length) {
            robotState.setState(State.IDLE);
            return true;
        }

        if (Timer.getFPGATimestamp() - this.startTime >= revLength) {
            robotState.setState(State.SUBWOOFER);
        }

        return false;
    }

}
