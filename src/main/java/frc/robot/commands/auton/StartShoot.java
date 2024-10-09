package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.Intake.IntakeState;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.Shooter.ShooterState;

public class StartShoot extends Command {
    private final Shooter shooter;
    private final Intake intake;
    private double startTime;

    public StartShoot(Shooter shooter, Intake intake) {
        this.shooter = shooter;
        this.intake = intake;
    }

    @Override
    public void initialize(){
        this.startTime = Timer.getFPGATimestamp();
        shooter.setState(ShooterState.SUBWOOFER_REVVING);
    }

    @Override
    public boolean isFinished() {
        if (Timer.getFPGATimestamp() - this.startTime > 3.0) {
            shooter.setState(ShooterState.IDLE);
            intake.setState(IntakeState.IDLE);
            return true;
        }

        if (Timer.getFPGATimestamp() - this.startTime > 2.0) {
            shooter.setState(ShooterState.SUBWOOFER);
            intake.setState(IntakeState.SHOOTING);
        }

        return false;
    }

}
