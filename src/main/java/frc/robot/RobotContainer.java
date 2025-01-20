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
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AnkitPoint;
import frc.robot.subsystems.Drivetrain.Drivetrain;
import frc.robot.subsystems.Drivetrain.Drivetrain.DriveState;
import frc.robot.subsystems.Drivetrain.SwerveDrivetrainIO.SwerveDrivetrainIONeo;
import frc.robot.subsystems.Drivetrain.SwerveDrivetrainIO.SwerveDrivetrainIOSim;
import frc.robot.subsystems.Limelight.Limelight;
import frc.robot.subsystems.Limelight.LimelightIO.LimelightIO;
import frc.robot.subsystems.Limelight.LimelightIO.LimelightIOLimelight3;
import frc.robot.subsystems.Limelight.LimelightIO.LimelightIOPhoton;
import frc.robot.utils.LoggedTunableNumber;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

  public static final double kTrackWidthX = Units.inchesToMeters(15.25);
  public static final double kTrackWidthY = Units.inchesToMeters(16.25);
  public static final double kDriveBaseRadius = Math.hypot(kTrackWidthX / 2.0, kTrackWidthY / 2.0);

  private Limelight limelight;
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

    if (Constants.getMode() == Constants.Mode.REAL && false) {
      limelightIO =
          new LimelightIOLimelight3(NetworkTableInstance.getDefault().getTable("limelight"));
    } else {
      limelightIO = new LimelightIOPhoton("limelight", Constants.kLimelightRobotToCamera);
    }

    limelight = new Limelight(limelightIO);

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
        new RobotState(drivetrain, Ports.driverControllerPort, Ports.operatorControllerPort);

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
  }

  public RobotContainer() {
    initializeSubsystems();
    configureBindings();
  }

  public RobotState getRobotState() {
    return robotState;
  }

  public Command getAutonomousCommand() {
    return Commands.none();
  }

  public Drivetrain getDrivetrain() {
    return drivetrain;
  }
}
