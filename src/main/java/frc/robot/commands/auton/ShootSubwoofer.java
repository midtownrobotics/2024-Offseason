package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.Intake.IntakeState;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.Shooter.ShooterState;

public class ShootSubwoofer extends Command {
    private final Shooter shooter;
    private final Intake intake;
    private double startTime;

    public ShootSubwoofer(Shooter shooter, Intake intake) {
        this.shooter = shooter;
        this.intake = intake;
        addRequirements(shooter);
        addRequirements(intake);
    }

    @Override
    public void initialize(){
        this.startTime = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isFinished() {
        double length = Constants.AutonConstants.AUTON_SHOOT_SUBWOOFER_LENGTH_SEC.get();

        if (Timer.getFPGATimestamp() - this.startTime >= length) {
            shooter.setState(ShooterState.IDLE);
            intake.setState(IntakeState.IDLE);
            return true;
        }

        if (Timer.getFPGATimestamp() - this.startTime < length) {
            shooter.setState(ShooterState.SUBWOOFER);
            intake.setState(IntakeState.SHOOTING);
        }

        return false;
    }

}
