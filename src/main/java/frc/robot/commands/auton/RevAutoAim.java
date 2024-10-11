package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.Intake.IntakeState;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.Shooter.ShooterState;

public class RevAutoAim extends Command {
    private final Shooter shooter;
    private final Intake intake;

    public RevAutoAim(Shooter shooter, Intake intake) {
        this.shooter = shooter;
        this.intake = intake;
        addRequirements(shooter);
        addRequirements(intake);
    }

    @Override
    public void initialize(){
        intake.setState(IntakeState.IDLE);
        shooter.setState(ShooterState.AUTO_AIM_REVVING);
    }

}
