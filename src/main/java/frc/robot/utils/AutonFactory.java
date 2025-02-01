package frc.robot.utils;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.RobotState.State;
import frc.robot.commands.WheelRadiusCharacterization;
import frc.robot.commands.WheelRadiusCharacterization.Direction;
import frc.robot.commands.auton.AlignWithSpeaker;
import frc.robot.subsystems.Drivetrain.Drivetrain;
import frc.robot.subsystems.Drivetrain.Drivetrain.DriveState;
import frc.robot.subsystems.Limelight.Limelight;
import frc.robot.subsystems.Shooter.Shooter.ShooterState;
import org.littletonrobotics.junction.Logger;

public class AutonFactory {

  private final Drivetrain m_drivetrain;
  private final RobotState m_robotState;
  private final Limelight m_limelight;
  private final SendableChooser<Command> m_autoChooser;

  private LoggedTunableNumber PATHPLANNER_TRANSLATION_P =
      new LoggedTunableNumber("PathPlanner/Translation_P", 1.0);
  private LoggedTunableNumber PATHPLANNER_TRANSLATION_I =
      new LoggedTunableNumber("PathPlanner/Translation_I", 0);
  private LoggedTunableNumber PATHPLANNER_TRANSLATION_D =
      new LoggedTunableNumber("PathPlanner/Translation_D", 0);
  private LoggedTunableNumber PATHPLANNER_ROTATION_P =
      new LoggedTunableNumber("PathPlanner/ROTATION_P", 1.0);
  private LoggedTunableNumber PATHPLANNER_ROTATION_I =
      new LoggedTunableNumber("PathPlanner/ROTATION_I", 0);
  private LoggedTunableNumber PATHPLANNER_ROTATION_D =
      new LoggedTunableNumber("PathPlanner/ROTATION_D", 0);

  public AutonFactory(RobotState robotState, Drivetrain drivetrain, Limelight limelight) {
    this.m_drivetrain = drivetrain;
    this.m_robotState = robotState;
    this.m_limelight = limelight;

    registerNamedCommands();

    PPHolonomicDriveController controller =
        new PPHolonomicDriveController(new PIDConstants(1.0), new PIDConstants(1.0));
    RobotConfig robotConfig = null;
    try {
      robotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
    }

    AutoBuilder.configure(
        m_drivetrain::getPose,
        m_drivetrain::resetOdometry,
        m_drivetrain::getRobotRelativeSpeeds,
        m_drivetrain::setPathPlannerDesired,
        controller,
        robotConfig,
        () -> false,
        m_drivetrain // Reference to this subsystem to set requirements
        );

    m_autoChooser = AutoBuilder.buildAutoChooser();
    m_autoChooser.addOption(
        "Wheel Radius Characterization",
        new WheelRadiusCharacterization(m_drivetrain, Direction.COUNTER_CLOCKWISE));
    SmartDashboard.putData(m_autoChooser);

    PathPlannerLogging.setLogCurrentPoseCallback(
        (pose) -> Logger.recordOutput("PathPlanner/currentPose", pose));
    PathPlannerLogging.setLogTargetPoseCallback(
        (pose) -> Logger.recordOutput("PathPlanner/nextPose", pose));
  }

  public Command getAutonCommand() {
    return m_autoChooser.getSelected();
  }

  public void registerNamedCommands() {
    if (Robot.isSimulation()) return;
    NamedCommands.registerCommand(
        "Idle",
        new InstantCommand(
            () -> {
              m_robotState.setState(State.IDLE);
            }));

    NamedCommands.registerCommand(
        "Rev",
        new InstantCommand(
            () -> {
              m_robotState.setState(State.SUBWOOFER_REVVING);
            }));

    NamedCommands.registerCommand(
        "SubwooferRev", new InstantCommand(() -> m_robotState.setState(State.SUBWOOFER_REVVING)));

    NamedCommands.registerCommand(
        "SubwooferShoot",
        new FunctionalCommand(
                () -> m_robotState.setState(State.SUBWOOFER_REVSHOOT),
                () -> {},
                (interrupted) -> {},
                () -> {
                  return m_robotState.getShooterState() == ShooterState.SUBWOOFER;
                })
            .andThen(
                new WaitCommand(0.1)
                    .andThen(
                        new InstantCommand(() -> m_robotState.setState(State.INTAKE_REVVING)))));

    NamedCommands.registerCommand(
        "Intake",
        new InstantCommand(
            () -> {
              m_robotState.setState(State.INTAKE_REVVING);
            }));

    NamedCommands.registerCommand(
        "EnableVision",
        new InstantCommand(
            () -> {
              m_limelight.setAutonVisionEnabled(true);
            }));

    NamedCommands.registerCommand(
        "DisableVision",
        new InstantCommand(
            () -> {
              m_limelight.setAutonVisionEnabled(false);
            }));

    NamedCommands.registerCommand("AutoAimRotate", new AlignWithSpeaker(m_limelight, m_drivetrain));

    NamedCommands.registerCommand(
        "AutoAimRevShoot",
        new FunctionalCommand(
                () -> m_robotState.setState(State.AUTO_AIM_REVSHOOT),
                () -> {},
                (interrupted) -> {},
                () -> {
                  return m_robotState.getShooterState() == ShooterState.AUTO_AIM;
                })
            .andThen(
                new WaitCommand(0.1)
                    .andThen(
                        new InstantCommand(() -> m_robotState.setState(State.INTAKE_REVVING)))));

    NamedCommands.registerCommand(
        "EnableAlignedFollowPath",
        new InstantCommand(
            () -> {
              m_drivetrain.setState(DriveState.FOLLOW_PATH_ALIGNED);
            }));
  }
}
