package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.Intake.IntakeState;

public class StartIntake extends Command {
    private final Intake intake;

    public StartIntake(Intake intake) {
        this.intake = intake;
    }

    @Override
    public void initialize() {
        intake.setState(IntakeState.INTAKING);
    }
}
