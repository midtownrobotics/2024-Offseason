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
import frc.robot.Ports.ShooterPorts;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.BeamBreakIO.BeamBreakIO;
import frc.robot.subsystems.Intake.BeamBreakIO.BeamBreakIODIO;
import frc.robot.subsystems.Intake.BeamBreakIO.BeamBreakIOSim;
import frc.robot.subsystems.Intake.Roller.RollerIO;
import frc.robot.subsystems.Intake.Roller.RollerIONeo;
import frc.robot.subsystems.Intake.Roller.RollerIOSim;
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
import frc.robot.subsystems.drivetrain.DrivetrainInterface;
import frc.robot.subsystems.drivetrain.NeoSwerveDrive.NeoSwerveDrivetrain;

public class RobotContainer {

  private Climber climber = new Climber();
  private Shooter shooter;
  private Intake intake;

  private DrivetrainInterface drivetrain;

  private RobotState robotState;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController driver = new CommandXboxController(0); // My joystick
  // private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  // private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
  //     .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
  //     .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  // private final Telemetry logger = new Telemetry(MaxSpeed);

  public static double deadzone(double a, double b, double c, double zone) {
		if (Math.sqrt(Math.pow(a, 2)+Math.pow(b, 2)+Math.pow(c, 2)) > zone) {
			return a * Math.abs(a);
		} else {
			return 0;
		}
	}

  private void configureBindings() {

    drivetrain.configureDefaultCommand(driver);

    driver.a().onTrue(new InstantCommand(() -> drivetrain.resetHeading()));
    driver.x().onTrue(new InstantCommand(() -> {}));

  }

  public void initializeSubsystems() {

    // Drivetrain

    if (Constants.USE_KRAKEN_DRIVETRAIN.get()) { // default value is false which means neo is used
      drivetrain = TunerConstants.DriveTrain;
    } else {
      drivetrain = new NeoSwerveDrivetrain();
    }
    
    // Climber

    climber = new Climber();

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

    intake = new Intake(rollerIO, beamBreakIO);

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
