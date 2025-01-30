// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Ports.IntakePorts;
import frc.robot.Ports.ShooterPorts;
import frc.robot.commands.AnkitPoint;
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
import frc.robot.subsystems.Intake.Intake.IntakeState;
import frc.robot.subsystems.Intake.Roller.RollerIO;
import frc.robot.subsystems.Intake.Roller.RollerIONeo;
import frc.robot.subsystems.Intake.Roller.RollerIOSim;
import frc.robot.subsystems.Limelight.Limelight;
import frc.robot.subsystems.Limelight.LimelightIO.LimelightIO;
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
import frc.robot.subsystems.Shooter.Shooter.ShooterState;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.utils.AutonFactory;
import frc.robot.utils.LoggedTunableNumber;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

  public static final double kTrackWidthX = Units.inchesToMeters(15.25);
  public static final double kTrackWidthY = Units.inchesToMeters(16.25);
  public static final double kDriveBaseRadius = Math.hypot(kTrackWidthX / 2.0, kTrackWidthY / 2.0);

  private Climber climber;
  private Shooter shooter;
  private Intake intake;
  private BeamBreak beamBreak;
  private Limelight limelight;
  private Vision vision;
  private AutonFactory m_autonFactory;
  private LoggedDashboardChooser<String> drivingMode;

  private Drivetrain drivetrain;

  private RobotState robotState;

  private AprilTagFieldLayout m_aprilTagFieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

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

  private boolean operatorPovUp = false;
  private boolean operatorPovDown = false;

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
              double pigeonValue = drivetrain.getAngle();
              if (drivingMode != null
                  && drivingMode.get() != null
                  && drivingMode.get().equals("robot")) {
                pigeonValue = 0;
              }

              drivetrain.setDriverDesired(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      driverX, driverY, driverRot, Rotation2d.fromDegrees(pigeonValue)));
            },
            drivetrain));

    driver.a().onTrue(new InstantCommand(() -> drivetrain.resetHeading()));

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
                () -> drivetrain.setState(DriveState.SPEAKER_AUTO_ALIGN), drivetrain))
        .onFalse(new InstantCommand(() -> drivetrain.setState(DriveState.MANUAL), drivetrain));

    driver
        .leftTrigger()
        .whileTrue(
            new StartEndCommand(() -> drivetrain.setBoost(true), () -> drivetrain.setBoost(false)));

    LoggedTunableNumber targetX = new LoggedTunableNumber("Drive/AnkitPoint/TargetX", 1);
    LoggedTunableNumber targetY = new LoggedTunableNumber("Drive/AnkitPoint/TargetY", 0);
    LoggedTunableNumber targetYaw = new LoggedTunableNumber("Drive/AnkitPoint/TargetYaw", 0);
    driver
        .b()
        .onTrue(
            new InstantCommand(() -> drivetrain.setState(DriveState.DRIVE_TO_POINT), drivetrain))
        .onFalse(new InstantCommand(() -> drivetrain.setState(DriveState.MANUAL), drivetrain))
        /** "x": -0.038099999999999995, "y": 5.547867999999999, */
        .whileTrue(
            new AnkitPoint(
                drivetrain,
                () ->
                    m_aprilTagFieldLayout
                        .getTagPose(2)
                        .get()
                        .toPose2d()
                        .transformBy(
                            new Transform2d(
                                new Translation2d(targetX.get(), targetY.get()),
                                new Rotation2d()))));
    // .whileTrue(
    //   AutoBuilder.pathfindToPose(new Pose2d(
    //     new Translation2d(-0.038099999999999995, 5.547867999999999).plus(new Translation2d(1,
    // 0)),
    //     new Rotation2d()
    //     ), new PathConstraints(1, 1, .5, .5))
    // );

    operator
        .povUp()
        .whileTrue(new StartEndCommand(() -> operatorPovUp = true, () -> operatorPovUp = false));

    operator
        .povDown()
        .whileTrue(
            new StartEndCommand(() -> operatorPovDown = true, () -> operatorPovDown = false));

    operator
        .rightBumper()
        .whileTrue(
            new StartEndCommand(
                () -> {
                  if (intake.currentSetState != IntakeState.NOTE_HELD) {
                    intake.setState(IntakeState.INTAKING);
                  }
                },
                () -> {
                  if (intake.currentSetState != IntakeState.NOTE_HELD) {
                    intake.setState(IntakeState.IDLE);
                  }
                },
                intake));
    // operator.leftBumper().onTrue(new InstantCommand(() -> {
    //     // intake.setState(IntakeState.GRITS_FEEDING);
    //     shooter.setState(ShooterState.GRITS_FEEDING_REVVING);
    //   }), shooter)
    //   .onFalse(new InstantCommand(() -> {
    //     shooter.setState(ShooterState.IDLE);
    //   }), shooter);
    // operator
    //     .leftBumper()
    //     .whileTrue(
    //         new StartEndCommand(
    //             () -> {
    //               if (robotState.currentState != State.NOTE_HELD) {
    //                 robotState.setState(State.INTAKE_REVVING);
    //               } else {
    //                 robotState.setState(State.SUBWOOFER_REVVING);
    //               }
    //             },
    //             () -> {
    //               if (robotState.currentState != State.NOTE_HELD) {
    //                 robotState.setState(State.IDLE);
    //               }
    //             },
    //             intake));
    // operator
    //     .leftTrigger()
    //     .whileTrue(
    //         new StartEndCommand(
    //             () -> robotState.setState(State.VOMITING),
    //             () -> robotState.setState(State.IDLE),
    //             intake,
    //             shooter));
    operator
        .leftBumper()
        .whileTrue(
            new StartEndCommand(
                () -> {
                  intake.setState(IntakeState.VOMITING);
                  shooter.setState(ShooterState.VOMITING);
                },
                () -> {
                  intake.setState(IntakeState.IDLE);
                  shooter.setState(ShooterState.IDLE);
                },
                intake,
                shooter));

    operator
        .leftTrigger()
        .whileTrue(
            new StartEndCommand(
                () -> {
                  intake.setState(IntakeState.VOMITING);
                  shooter.setState(ShooterState.VOMITING);
                },
                () -> {
                  intake.setState(IntakeState.IDLE);
                  shooter.setState(ShooterState.IDLE);
                },
                intake,
                shooter));

    operator
        .rightTrigger()
        .whileTrue(
            new StartEndCommand(
                () -> {
                  switch (shooter.currentState) {
                    case SUBWOOFER_REVVING:
                      shooter.setState(ShooterState.SUBWOOFER);
                      intake.setState(IntakeState.SHOOTING);
                      break;
                    case AMP_REVVING:
                      shooter.setState(ShooterState.AMP);
                      intake.setState(IntakeState.SHOOTING);
                      break;
                    case AUTO_AIM_REVVING:
                      shooter.setState(ShooterState.AUTO_AIM);
                      intake.setState(IntakeState.SHOOTING);
                      break;
                    default:
                      break;
                  }
                },
                () -> {
                  intake.setState(IntakeState.IDLE);
                  switch (shooter.currentState) {
                    case SUBWOOFER:
                      shooter.setState(ShooterState.SUBWOOFER_REVVING);
                      break;
                    case AMP:
                      shooter.setState(ShooterState.AMP_REVVING);
                      break;
                    case AUTO_AIM:
                      shooter.setState(ShooterState.AUTO_AIM_REVVING);
                      break;
                    default:
                      break;
                  }
                },
                shooter,
                intake));

    operator
        .a()
        .whileTrue(
            new InstantCommand(() -> shooter.setState(ShooterState.SUBWOOFER_REVVING), shooter));
    operator
        .x()
        .whileTrue(new InstantCommand(() -> shooter.setState(ShooterState.AMP_REVVING), shooter));
    operator
        .y()
        .whileTrue(
            new InstantCommand(() -> shooter.setState(ShooterState.AUTO_AIM_REVVING), shooter));
    operator.b().whileTrue(new InstantCommand(() -> shooter.setState(ShooterState.IDLE), shooter));
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

    LimelightIO limelightIO = new LimelightIOSim();
    VisionIO visionIO;

    if (Constants.getMode() == Constants.Mode.REAL && false) {
      // limelightIO = new
      // LimelightIOLimelight3(NetworkTableInstance.getDefault().getTable("limelight"));
      // visionIO = new VisionIOLimelight(null, null)
    } else {
      // limelightIO = new LimelightIOPhoton("limelight", Constants.kLimelightRobotToCamera);
      visionIO =
          new VisionIOPhotonVision(
              VisionConstants.kLimelightName, VisionConstants.kLimelightRobotToCamera);
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

    // Beam break
    BeamBreakIO beamBreakIO;

    if (Constants.getMode() == Constants.Mode.REAL) {
      beamBreakIO = new BeamBreakIODIO(IntakePorts.beamBreak);
    } else {
      beamBreakIO = new BeamBreakIOSim();
    }

    beamBreak = new BeamBreak(beamBreakIO);

    // Intake

    RollerIO rollerIO;

    if (Constants.getMode() == Constants.Mode.REAL) {
      rollerIO = new RollerIONeo(IntakePorts.runExternal, IntakePorts.runInternal);
    } else {
      rollerIO = new RollerIOSim();
    }

    intake = new Intake(rollerIO, beamBreak);

    // Climber

    ClimberIO climberIO;

    if (Constants.getMode() == Constants.Mode.REAL) {
      climberIO =
          new ClimberIONeo(Ports.ClimberPorts.leftClimberID, Ports.ClimberPorts.rightClimberID);
    } else {
      climberIO = new ClimberIOSim();
    }

    climber = new Climber(operator, climberIO);

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
    robotState =
        new RobotState(
            shooter,
            climber,
            intake,
            drivetrain,
            beamBreak,
            Ports.driverControllerPort,
            Ports.operatorControllerPort);

    drivingMode = new LoggedDashboardChooser<String>("Driving Mode");
    drivingMode.addDefaultOption("Field Relative", "field");
    drivingMode.addOption("Robot Relative", "robot");

    // AutoBuilder.configureHolonomic(
    //   drivetrain::getPose,
    //   drivetrain::resetOdometry,
    //   drivetrain::getRobotRelativeSpeeds,
    //   drivetrain::setDriveToPointDesired,
    //   new HolonomicPathFollowerConfig(new PIDConstants(5), new PIDConstants(2.5, 0.06), 2,
    // kDriveBaseRadius, new ReplanningConfig()), () -> false, drivetrain);

    vision = new Vision(drivetrain::addVisionMeasurement, visionIO);
  }

  public RobotContainer() {
    initializeSubsystems();
    configureBindings();
  }

  public RobotState getRobotState() {
    return robotState;
  }

  public Command getAutonomousCommand() {
    return m_autonFactory.getAutonCommand();
  }

  public AutonFactory getAutonFactory() {
    return m_autonFactory;
  }

  public Drivetrain getDrivetrain() {
    return drivetrain;
  }

  public void onDriverStationConnected() {
    configureAutonomous();
  }

  public void configureAutonomous() {
    m_autonFactory = new AutonFactory(robotState, drivetrain, limelight);
  }
}
