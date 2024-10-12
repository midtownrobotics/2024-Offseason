package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake.Roller.RollerIO;
import frc.robot.subsystems.Intake.Roller.RollerIOInputsAutoLogged;

public class Intake extends SubsystemBase{

    private RollerIO rollerIO;
    private RollerIOInputsAutoLogged rollerIOInputs = new RollerIOInputsAutoLogged();

    public enum IntakeState {
        INTAKING,
        VOMITING,
        TUNING,
        IDLE,
        SHOOTING
    }

    private IntakeState currentSetState = IntakeState.IDLE;

    public Intake(RollerIO rollerIO) {
        this.rollerIO = rollerIO;
    }

    public void setState(IntakeState to) {
        currentSetState = to;
    }
 
    @Override
    public void periodic() {

        Logger.recordOutput("Intake/State", currentSetState.toString());

        switch (currentSetState) {
            case INTAKING:
                rollerIO.setSpeed(1);
                break;
            case SHOOTING:
                rollerIO.setSpeed(1);
                break;
            case VOMITING:
                rollerIO.setSpeed(-1);
                break;
            case TUNING:
                // TODO
                break;
            case IDLE:
                rollerIO.setSpeed(0);
                break;
            default:
                break;
        }
        
        rollerIO.updateInputs(rollerIOInputs);
        Logger.processInputs("Intake/Roller", rollerIOInputs);
    }
}
