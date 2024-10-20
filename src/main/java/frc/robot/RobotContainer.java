// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.Instant;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Ports.IntakePorts;
import frc.robot.Ports.ShooterPorts;
// import frc.robot.generated.TunerConstants;
import frc.robot.RobotState.State;
import frc.robot.subsystems.BeamBreak.BeamBreak;
import frc.robot.subsystems.BeamBreak.BeamBreakIO.BeamBreakIO;
import frc.robot.subsystems.BeamBreak.BeamBreakIO.BeamBreakIODIO;
import frc.robot.subsystems.BeamBreak.BeamBreakIO.BeamBreakIOSim;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Climber.ClimberIO.ClimberIO;
import frc.robot.subsystems.Climber.ClimberIO.ClimberIONeo;
import frc.robot.subsystems.Climber.ClimberIO.ClimberIOSim;
import frc.robot.subsystems.Drivetrain.Drivetrain;
import frc.robot.subsystems.Drivetrain.Drivetrain.DriveState;
import frc.robot.subsystems.Drivetrain.SwerveDrivetrainIO.SwerveDrivetrainIONeo;
import frc.robot.subsystems.Drivetrain.SwerveDrivetrainIO.SwerveDrivetrainIOSim;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.Roller.RollerIO;
import frc.robot.subsystems.Intake.Roller.RollerIONeo;
import frc.robot.subsystems.Intake.Roller.RollerIOSim;
import frc.robot.subsystems.Limelight.Limelight;
import frc.robot.subsystems.Limelight.LimelightIO.LimelightIO;
import frc.robot.subsystems.Limelight.LimelightIO.LimelightIOLimelight3;
import frc.robot.subsystems.Limelight.LimelightIO.LimelightIOSim;
import frc.robot.subsystems.Shooter.Feeder.FeederIO;
import frc.robot.subsystems.Shooter.Feeder.FeederIONeo;
import frc.robot.subsystems.Shooter.Feeder.FeederIOSim;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelIO;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelIONeo;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelIOSim;
import frc.robot.subsystems.Shooter.Pivot.PivotIO;
import frc.robot.subsystems.Shooter.Pivot.PivotIONeo;
import frc.robot.subsystems.Shooter.Pivot.PivotIOSim;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.utils.AutonFactory;

public class RobotContainer {

  private Climber climber;
  private Shooter shooter;
  private Intake intake;
  private BeamBreak beamBreak;
  private Limelight limelight;
  private AutonFactory m_autonFactory;

  private Drivetrain drivetrain;

  private RobotState robotState;

  private final CommandXboxController driver =
      new CommandXboxController(Ports.driverControllerPort);
  private final CommandXboxController operator =
      new CommandXboxController(Ports.operatorControllerPort);

  public static double deadzone(double a, double b, double c, double zone) {
    if (Math.sqrt(Math.pow(a, 2) + Math.pow(b, 2) + Math.pow(c, 2)) > zone) {
      return a * Math.abs(a);
    } else {
      return 0;
    }
  }

  private void configureBindings() {

    drivetrain.setDefaultCommand(
        new RunCommand(
            () -> {
              double driverX =
                  RobotContainer.deadzone(
                          -driver.getLeftY(),
                          -driver.getLeftX(),
                          -driver.getRightX(),
                          Constants.JOYSTICK_THRESHOLD)
                      * Constants.CONTROL_LIMITER;
              double driverY =
                  RobotContainer.deadzone(
                          -driver.getLeftX(),
                          -driver.getLeftY(),
                          -driver.getRightX(),
                          Constants.JOYSTICK_THRESHOLD)
                      * Constants.CONTROL_LIMITER;
              double driverRot =
                  RobotContainer.deadzone(
                          -driver.getRightX(),
                          -driver.getLeftY(),
                          -driver.getLeftX(),
                          Constants.JOYSTICK_THRESHOLD)
                      * Constants.CONTROL_LIMITER;

              // drivetrain.setDriverDesired(
              //   driverX, driverY, driverRot
              // );
              drivetrain.setDriverDesired(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      driverX, driverY, driverRot, Rotation2d.fromDegrees(drivetrain.getAngle())));
            },
            drivetrain));

    climber.setDefaultCommand(
        new RunCommand(
            () -> {
              double operatorLeft = Constants.deadzone(-operator.getLeftY());
              double operatorRight = Constants.deadzone(-operator.getRightY());
              climber.setPower(operatorLeft, operatorRight);
            },
            climber));

    driver.a().onTrue(new InstantCommand(() -> drivetrain.resetHeading()));
    // Note: These probably do not need to require drivetrain
    driver
        .x()
        .whileTrue(
            new StartEndCommand(
                () -> drivetrain.setState(DriveState.X),
                () -> drivetrain.setState(DriveState.MANUAL),
                drivetrain));
    driver
        .y()
        .onTrue(
            new InstantCommand(
                () -> drivetrain.setState(DriveState.SPEAKER_AUTO_ALIGN),
                drivetrain
            )
        )
        .onFalse(new InstantCommand(
                () -> drivetrain.setState(DriveState.MANUAL),
                drivetrain
            )
        );

    driver
        .leftTrigger()
        .whileTrue(
            new StartEndCommand(() -> drivetrain.setBoost(true), () -> drivetrain.setBoost(false)));

    operator
        .rightBumper()
        .whileTrue(
            new StartEndCommand(
                () -> {
                  if (robotState.currentState != State.NOTE_HELD) {
                    robotState.setState(State.INTAKING);
                  }
                },
                () -> {
                  if (robotState.currentState != State.NOTE_HELD) {
                    robotState.setState(State.IDLE);
                  }
                },
                intake));

    operator
        .leftBumper()
        .whileTrue(
            new StartEndCommand(
                () -> robotState.setState(State.VOMITING),
                () -> robotState.setState(State.IDLE),
                intake,
                shooter));
    operator
        .leftTrigger()
        .whileTrue(
            new StartEndCommand(
                () -> robotState.setState(State.VOMITING),
                () -> robotState.setState(State.IDLE),
                intake,
                shooter));

    operator
        .rightTrigger()
        .whileTrue(
            new StartEndCommand(
                () -> {
                  switch (robotState.currentState) {
                    case SUBWOOFER_REVVING:
                      robotState.setState(State.SUBWOOFER);
                      break;
                    case AMP_REVVING:
                      robotState.setState(State.AMP);
                      break;
                    case AUTO_AIM_REVVING:
                      robotState.setState(State.AUTO_AIM);
                      break;
                    default:
                      break;
                  }
                },
                () -> robotState.setState(State.IDLE),
                shooter,
                intake));

    operator
        .a()
        .whileTrue(new InstantCommand(() -> robotState.setState(State.SUBWOOFER_REVVING), shooter));
    operator
        .x()
        .whileTrue(new InstantCommand(() -> robotState.setState(State.AMP_REVVING), shooter));
    operator
        .y()
        .whileTrue(new InstantCommand(() -> robotState.setState(State.AUTO_AIM_REVVING), shooter));
    operator
        .b()
        .whileTrue(new InstantCommand(() -> robotState.setState(State.IDLE), shooter, intake));
    // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    // joystick.b().whileTrue(drivetrain
    //     .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(),
    // -joystick.getLeftX()))));

    // if (Utils.isSimulation()) {
    //   drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    // }
    // drivetrain.registerTelemetry(logger::telemeterize);

  }

  public void initializeSubsystems() {

    // Limelight

    LimelightIO limelightIO;

    if (Constants.getMode() == Constants.Mode.REAL) {
      limelightIO =
          new LimelightIOLimelight3(NetworkTableInstance.getDefault().getTable("limelight"));
    } else {
      limelightIO = new LimelightIOSim();
    }

    limelight = new Limelight(limelightIO);

    // Shooter

    FlywheelIO flywheelIO;
    PivotIO pivotIO;
    FeederIO feederIO;

    if (Constants.getMode() == Constants.Mode.REAL) {
      flywheelIO = new FlywheelIONeo(ShooterPorts.leftFlywheel, ShooterPorts.rightFlywheel);
      pivotIO = new PivotIONeo(ShooterPorts.pivot, ShooterPorts.pivotEncoder);
      feederIO = new FeederIONeo(ShooterPorts.rollerTop, ShooterPorts.rollerBottom);
    } else {
      flywheelIO = new FlywheelIOSim();
      pivotIO = new PivotIOSim();
      feederIO = new FeederIOSim();
    }

    shooter = new Shooter(flywheelIO, pivotIO, feederIO, limelight);

    // Intake

    RollerIO rollerIO;

    if (Constants.getMode() == Constants.Mode.REAL) {
      rollerIO = new RollerIONeo(IntakePorts.runExternal, IntakePorts.runInternal);
    } else {
      rollerIO = new RollerIOSim();
    }

    intake = new Intake(rollerIO);

    // Climber

    ClimberIO climberIO;

    if (Constants.getMode() == Constants.Mode.REAL) {
      climberIO =
          new ClimberIONeo(Ports.ClimberPorts.leftClimberID, Ports.ClimberPorts.rightClimberID);
    } else {
      climberIO = new ClimberIOSim();
    }

    climber = new Climber(operator, climberIO);

    // BeamBreak

    BeamBreakIO beamBreakIO;

    if (Constants.getMode() == Constants.Mode.REAL) {
      beamBreakIO = new BeamBreakIODIO(IntakePorts.beamBreak);
    } else {
      beamBreakIO = new BeamBreakIOSim();
    }

    beamBreak =
        new BeamBreak(
            beamBreakIO, robotState, Ports.driverControllerPort, Ports.operatorControllerPort);

    // Drivetrain

    // if (Constants.USE_KRAKEN_DRIVETRAIN.get()) { // default value is false which means neo is
    // used
    //   drivetrain = TunerConstants.DriveTrain;
    // } else {
    // drivetrain = new NeoSwerveDrivetrain();
    // }
    if (Robot.isSimulation()) {
      drivetrain = new Drivetrain(new SwerveDrivetrainIOSim(), limelight);
    } else {
      drivetrain = new Drivetrain(new SwerveDrivetrainIONeo(), limelight);
    }

    // Robot State
    robotState = new RobotState(shooter, climber, intake, drivetrain);
    beamBreak.setRobotState(robotState);
  }

  public RobotContainer() {
    initializeSubsystems();
    configureBindings();
  }

  public RobotState getRobotState() {
    return robotState;
  }

  public Command getAutonomousCommand() {
    return m_autonFactory
        .getAutonCommand()
        .andThen(
            () -> {
              drivetrain.setState(DriveState.X);
              robotState.setState(State.IDLE);
            },
            drivetrain);
  }

  public void onDriverStationConnected() {
    configureAutonomous();
  }

  public void configureAutonomous() {
    m_autonFactory = new AutonFactory(robotState, drivetrain, limelight);
  }
}
