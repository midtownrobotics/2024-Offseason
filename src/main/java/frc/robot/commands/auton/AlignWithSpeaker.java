package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.Constants.AutonConstants;
import frc.robot.subsystems.Drivetrain.Drivetrain;
import frc.robot.subsystems.Drivetrain.Drivetrain.DriveState;
import frc.robot.subsystems.Limelight.Limelight;

public class AlignWithSpeaker extends Command {
  private final Limelight limelight;
  private final Drivetrain drivetrain;

  public AlignWithSpeaker(Limelight limelight, Drivetrain drivetrain) {
    this.limelight = limelight;
    this.drivetrain = drivetrain;
  }

  @Override
  public void initialize() {
    drivetrain.setState(DriveState.SPEAKER_AUTO_ALIGN);
  }

  @Override
  public boolean isFinished() {
    double tolerance = AutonConstants.AUTO_AIM_TOLERANCE.get();
    return Math.abs(limelight.getTx()) < tolerance;
  }
}
