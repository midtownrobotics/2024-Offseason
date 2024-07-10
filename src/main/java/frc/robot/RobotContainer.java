// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Ports.IntakePorts;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Climber.ClimberIO.ClimberIO;
import frc.robot.Ports.ShooterPorts;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.BeamBreakIO.BeamBreakIO;
import frc.robot.subsystems.Intake.BeamBreakIO.BeamBreakIODIO;
import frc.robot.subsystems.Intake.BeamBreakIO.BeamBreakIOSim;
import frc.robot.subsystems.Intake.Roller.RollerIO;
import frc.robot.subsystems.Intake.Roller.RollerIONeo;
import frc.robot.subsystems.Intake.Roller.RollerIOSim;
import frc.robot.subsystems.NeoSwerveDrive.SwerveDrivetrain;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.Feeder.FeederIO;
import frc.robot.subsystems.Shooter.Feeder.FeederIONeo;
import frc.robot.subsystems.Shooter.Feeder.FeederIOSim;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelIO;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelIONeo;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelIOSim;
import frc.robot.subsystems.Shooter.Pivot.PivotIO;
import frc.robot.subsystems.Shooter.Pivot.PivotIONeo;
import frc.robot.subsystems.Shooter.Pivot.PivotIOSim;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.BeamBreakIO.BeamBreakIO;
import frc.robot.subsystems.Intake.BeamBreakIO.BeamBreakIODIO;
import frc.robot.subsystems.Intake.BeamBreakIO.BeamBreakIOSim;
import frc.robot.subsystems.Intake.Roller.RollerIO;
import frc.robot.subsystems.Intake.Roller.RollerIONeo;
import frc.robot.subsystems.Intake.Roller.RollerIOSim;
import frc.robot.subsystems.Climber.ClimberIO.ClimberIONeo;


public class RobotContainer {

  private Climber climber;
  private Shooter shooter;
  private Intake intake;

  private final SwerveDrivetrain drivetrain = new SwerveDrivetrain();

  private RobotState robotState;

  // private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  // private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1); // My joystick
  // private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain // My joystick
  // private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  // private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
  //     .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
  //     .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  // private final Telemetry logger = new Telemetry(MaxSpeed);
  // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  // private final Telemetry logger = new Telemetry(MaxSpeed);

  private static double deadzone(double a, double b, double c, double zone) {
		if (Math.sqrt(Math.pow(a, 2)+Math.pow(b, 2)+Math.pow(c, 2)) > zone) {
			return a * Math.abs(a);
		} else {
			return 0;
		}
	}

  private void configureBindings() {
    drivetrain.setDefaultCommand(new RunCommand(
			() -> drivetrain.drive(
				RobotContainer.deadzone(driver.getLeftY(), driver.getLeftX(), driver.getRightX(), Constants.JOYSTICK_THRESHOLD)*Constants.CONTROL_LIMITER,
				RobotContainer.deadzone(driver.getLeftX(), driver.getLeftY(), driver.getRightX(), Constants.JOYSTICK_THRESHOLD)*Constants.CONTROL_LIMITER,
				RobotContainer.deadzone(driver.getRightX(), driver.getLeftY(), driver.getLeftX(), Constants.JOYSTICK_THRESHOLD)*Constants.CONTROL_LIMITER,
		 	false), drivetrain));
    
    driver.a().onTrue(new InstantCommand(drivetrain::zeroHeading, drivetrain));
    // drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
    //     drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
    //                                                                                        // negative Y (forward)
    //         .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
    //         .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
    //     ));

    // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    // joystick.b().whileTrue(drivetrain
    //     .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // // reset the field-centric heading on left bumper press
    // joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    // if (Utils.isSimulation()) {
    //   drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    // }
    // drivetrain.registerTelemetry(logger::telemeterize);

  }

  public void initializeSubsystems() {
    
    // Climber
    ClimberIO climberIO;

    if (Constants.currentMode == Constants.Mode.REAL){
      climberIO = new ClimberIONeo(Ports.ClimberPorts.leftClimberID, Ports.ClimberPorts.rightClimberID);
    }else {
      // TODO: make the actual sim stuff for the climber and put it here
      climberIO = new ClimberIONeo(Ports.ClimberPorts.leftClimberID, Ports.ClimberPorts.rightClimberID);
    }

    climber = new Climber(operator, climberIO);

    // Shooter

    FlywheelIO flywheelIO;
    PivotIO pivotIO;
    FeederIO feederIO;

    if (Constants.currentMode == Constants.Mode.REAL) {
      flywheelIO = new FlywheelIONeo(ShooterPorts.leftFlywheel, ShooterPorts.rightFlywheel);
      pivotIO = new PivotIONeo(ShooterPorts.pivot, ShooterPorts.pivotEncoder);
      feederIO = new FeederIONeo(ShooterPorts.rollerTop, ShooterPorts.rollerBottom);
    } else {
      flywheelIO = new FlywheelIOSim();
      pivotIO = new PivotIOSim();
      feederIO = new FeederIOSim();
    }

    shooter = new Shooter(flywheelIO, pivotIO, feederIO);

    // Intake

    BeamBreakIO beamBreakIO;
    RollerIO rollerIO;

    if (Constants.currentMode == Constants.Mode.REAL) {
      beamBreakIO = new BeamBreakIODIO(IntakePorts.beamBreak);
      rollerIO = new RollerIONeo(IntakePorts.runExternal, IntakePorts.runInternal);
    } else {
      beamBreakIO = new BeamBreakIOSim();
      rollerIO = new RollerIOSim();
    }

    intake = new Intake(rollerIO, beamBreakIO, robotState);

    // Robot State

    robotState = new RobotState(shooter, climber, intake);
  }

  public RobotContainer() { 
    initializeSubsystems();
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
