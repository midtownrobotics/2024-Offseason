package frc.robot.subsystems.Drivetrain.SwerveDrivetrainIO;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class SwerveIOInputsAutoLogged extends SwerveDrivetrainIO.SwerveIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("Pose", pose);
    table.put("CurrentStates", currentStates);
    table.put("DesiredStates", desiredStates);
    table.put("PigeonYaw", pigeonYaw);
    table.put("State", state);
  }

  @Override
  public void fromLog(LogTable table) {
    pose = table.get("Pose", pose);
    currentStates = table.get("CurrentStates", currentStates);
    desiredStates = table.get("DesiredStates", desiredStates);
    pigeonYaw = table.get("PigeonYaw", pigeonYaw);
    state = table.get("State", state);
  }

  public SwerveIOInputsAutoLogged clone() {
    SwerveIOInputsAutoLogged copy = new SwerveIOInputsAutoLogged();
    copy.pose = this.pose;
    copy.currentStates = this.currentStates.clone();
    copy.desiredStates = this.desiredStates.clone();
    copy.pigeonYaw = this.pigeonYaw;
    copy.state = this.state;
    return copy;
  }
}
