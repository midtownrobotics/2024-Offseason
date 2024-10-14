package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.utils.LoggedTunableNumber;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public final class Constants {

    public static final double JOYSTICK_THRESHOLD = 0.1;
    public static final double CONTROL_LIMITER = 1;
	public static final double THEORETICAL_RESTING_VOLTAGE = 12;

  public static enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  public static Mode getMode() {
    if (RobotBase.isReal()) {
        return Mode.REAL;
    } else {
        return Mode.SIM;
    }
  }

  public static final boolean tuningMode = true;

  public static final class AutonConstants {
    public static LoggedDashboardNumber AUTON_SHOOT_SUBWOOFER_LENGTH_SEC =
        new LoggedDashboardNumber("Auton/Constants/AUTON_SHOOT_SUBWOOFER_LENGTH_SEC", 1.0);
    public static LoggedDashboardNumber AUTON_SHOOT_SUBWOOFER_REV_LENGTH_SEC =
        new LoggedDashboardNumber("Auton/Constants/AUTON_SHOOT_SUBWOOFER_REV_LENGTH_SEC", 1.0);
    public static LoggedDashboardNumber AUTON_SHOOT_AUTO_AIM_LENGTH_SEC =
        new LoggedDashboardNumber("Auton/Constants/AUTON_SHOOT_AUTO_AIM_LENGTH_SEC", 1.0);
  }

  public static final LoggedDashboardNumber deadzone =
      new LoggedDashboardNumber("CONTROLLER_DEADZONE", 0.1);

  public static final double deadzone(double input) {
    return Math.abs(input) > deadzone.get() ? input : 0;
  }

  public static final class IntakeConstants {
    public static LoggedDashboardNumber BEAMBREAK_DELAY =
        new LoggedDashboardNumber("Intake/Constants/BEAMBREAK_DELAY", 1);
    public static LoggedDashboardNumber CONTROLLER_RUMBLE_TIME =
        new LoggedDashboardNumber("Intake/Constants/CONTROLLER_RUMBLE_TIME", 12);

		public static double MOI = 0.00005; // TODO: Actually figure out what the MOI (an hour of erik explaing something to you that somehow ends with a vector)
		public static double GEARING = 5.26; // TODO: Figure this out too (should be on the side of the gearbox)
  }

  public static final class MotorConstants {
    public static int CURRENT_LIMIT_550 = 35;
    public static int CURRENT_LIMIT_1650 = 60;
  }

  // if true kraken drivetrain is used otherwise neo is used
  public static LoggedDashboardBoolean USE_KRAKEN_DRIVETRAIN =
      new LoggedDashboardBoolean("USE_KRAKEN_DRIVETRAIN", false);

  public static final class ShooterConstants {
    public static LoggedDashboardNumber INTAKING_ROLLER_VOLTAGE =
        new LoggedDashboardNumber("Shooter/Constants/INTAKING_ROLLER_VOLTAGE", 1.2);

    public static LoggedDashboardNumber SPEAKER_ANGLE =
        new LoggedDashboardNumber("Shooter/Constants/SPEAKER_ANGLE", 0.852);
    public static LoggedDashboardNumber SPEAKER_SPEED =
        new LoggedDashboardNumber("Shooter/Constants/SPEAKER_SPEED", 3100);
    public static LoggedDashboardNumber SPEAKER_ROLLER_VOLTAGE =
        new LoggedDashboardNumber("Shooter/Constants/SPEAKER_ROLLER_VOLTAGE", 12);

    public static LoggedDashboardNumber AMP_ANGLE =
        new LoggedDashboardNumber("Shooter/Constants/AMP_ANGLE", 0.812);
    public static LoggedDashboardNumber AMP_SPEED =
        new LoggedDashboardNumber("Shooter/Constants/AMP_SPEED", 430);
    public static LoggedDashboardNumber AMP_ROLLER_VOLTAGE =
        new LoggedDashboardNumber(
            "Shooter/Constants/AMP_ROLLER_VOLTAGE", AMP_SPEED.get() / 700 * 12);

    public static LoggedTunableNumber FLYWHEEL_SPEED_P =
        new LoggedTunableNumber("Shooter/Constants/FLYWHEEL_SPEED_P", 0.0005);
    public static LoggedTunableNumber FLYWHEEL_SPEED_I =
        new LoggedTunableNumber("Shooter/Constants/FLYWHEEL_SPEED_I", 0);
    public static LoggedTunableNumber FLYWHEEL_SPEED_D =
        new LoggedTunableNumber("Shooter/Constants/FLYWHEEL_SPEED_D", 0.0005);
    public static LoggedTunableNumber FLYWHEEL_SPEED_FF =
        new LoggedTunableNumber("Shooter/Constants/FLYWHEEL_SPEED_FF", 0.00024);

    public static LoggedTunableNumber PIVOT_P =
        new LoggedTunableNumber("Shooter/Constants/PIVOT_P", 10);
    public static LoggedTunableNumber PIVOT_I =
        new LoggedTunableNumber("Shooter/Constants/PIVOT_I", 0);
    public static LoggedTunableNumber PIVOT_D =
        new LoggedTunableNumber("Shooter/Constants/PIVOT_D", 0);

    public static LoggedDashboardNumber MIN_PIVOT_ANGLE =
        new LoggedDashboardNumber("Shooter/Constants/MIN_PIVOT_ANGLE", 0.81);
    public static LoggedDashboardNumber MAX_PIVOT_ANGLE =
        new LoggedDashboardNumber("Shooter/Constants/MAX_PIVOT_ANGLE", 0.99);
    
    public static LoggedDashboardNumber PIVOT_ANGLE_TOLERANCE = new LoggedDashboardNumber("Shooter/Constants/PIVOT_ANGLE_TOLERANCE", 0.02);
    public static LoggedDashboardNumber AMP_SPEED_TOLERANCE = new LoggedDashboardNumber("Shooter/Constants/AMP_SPEED_TOLERANCE", 15);
    public static LoggedDashboardNumber SPEAKER_SPEED_TOLERANCE = new LoggedDashboardNumber("Shooter/Constants/SUBWOOFER_SPEED_TOLERANCE", 50);

		public static final class Simulation {
			public static LoggedDashboardNumber FLYWHEEL_GEARING = new LoggedDashboardNumber("Shooter/Constants/Simulation/FLYWHEEL_GEARING", 5.26);
			public static LoggedDashboardNumber FLYWHEEL_MOI = new LoggedDashboardNumber("Shooter/Constants/Simulation/FLYWHEEL_MOI", .00005);

			public static LoggedDashboardNumber FEEDER_GEARING = new LoggedDashboardNumber("Shooter/Constants/Simulation/FEEDER_GEARING", 5.26);
			public static LoggedDashboardNumber FEEDER_MOI = new LoggedDashboardNumber("Shooter/Constants/Simulation/FEEDER_MOI", .00005);
		}
  }

  public static final class NeoDrivetrainConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double MAX_SPEED_METERS_PER_SECOND = 3.00; // SET FOR TESTING
    public static final double MAX_SPEED_METERS_PER_SECOND_BOOSTED =
        4.95; // NORMAl SPEED (USED TO BE SET TO ABOVE)
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND =
        10; // (Max 14.58) radians per second

    public static final double DIRECTION_SLEW_RATE = 1.2; // radians per second
    public static final double MAGNITUDE_SLEW_RATE =
        1.8; // 2.0; //1.8; // percent per second (1 = 100%)
    public static final double ROTATIONAL_SLEW_RATE =
        2.0; // 20.0; //2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double TRACK_WIDTH_METERS = 0.4826;

    // Distance between centers of right and left wheels on robot
    public static final double WHEEL_BASE_METERS = 0.4826;

    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics DRIVE_KINEMATICS =
        new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / 2),
            new Translation2d(WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2),
            new Translation2d(-WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / 2),
            new Translation2d(-WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2));

    // public static final AdvancedSwerveKinematics DRIVE_KINEMATICS_2 = new
    // AdvancedSwerveKinematics(
    // 		new Translation2d(WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / 2),
    // 		new Translation2d(WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2),
    // 		new Translation2d(-WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / 2),
    // 		new Translation2d(-WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2));

    public static final boolean kGyroReversed = false;
  }

  public static final class NeoSwerveModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double DRIVING_MOTOR_FREE_SPEED_RPS = 5676 /* <- Neo free speed RPM */ / 60;
    public static final double WHEEL_DIAMETER_METERS = 0.1016;
    public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the
    // bevel pinion
    public static final double DRIVING_MOTOR_REDUCTION =
        (45.0 * 17 * 50) / (kDrivingMotorPinionTeeth * 15 * 27);
    public static final double DRIVE_WHEEL_FREE_SPEED_RPS =
        (DRIVING_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE_METERS) / DRIVING_MOTOR_REDUCTION;

    public static final double DRIVING_ENCODER_POSITION_FACTOR_METERS_PER_ROTATION =
        (WHEEL_DIAMETER_METERS * Math.PI) / DRIVING_MOTOR_REDUCTION; // meters, per rotation
    public static final double DRIVING_ENCODER_VELOCITY_FACTOR_METERS_PER_SECOND_PER_RPM =
        ((WHEEL_DIAMETER_METERS * Math.PI) / DRIVING_MOTOR_REDUCTION)
            / 60.0; // meters per second, per RPM

    public static final double TURNING_MOTOR_REDUCTION =
        150.0 / 7.0; // ratio between internal relative encoder and Through Bore (or Thrifty in our
    // case) absolute encoder - 150.0 / 7.0

    public static final double TURNING_ENCODER_POSITION_FACTOR_RADIANS_PER_ROTATION =
        (2 * Math.PI) / TURNING_MOTOR_REDUCTION; // radians, per rotation
    public static final double TURNING_ENCODER_VELOCITY_FACTOR_RADIANS_PER_SECOND_PER_RPM =
        (2 * Math.PI) / TURNING_MOTOR_REDUCTION / 60.0; // radians per second, per RPM

    public static final double TURNING_ENCODER_POSITION_PID_MIN_INPUT_RADIANS = 0; // radians
    public static final double TURNING_ENCODER_POSITION_PID_MAX_INPUT_RADIANS =
        (2 * Math.PI); // radians

    public static final LoggedTunableNumber DRIVING_P =
        new LoggedTunableNumber("Drivetrain/constants/DRIVING_P", 0.04);
    public static final LoggedTunableNumber DRIVING_I =
        new LoggedTunableNumber("Drivetrain/constants/DRIVING_I", 0);
    public static final LoggedTunableNumber DRIVING_D =
        new LoggedTunableNumber("Drivetrain/constants/DRIVING_D", 0);
    public static final LoggedTunableNumber DRIVING_FF =
        new LoggedTunableNumber("Drivetrain/constants/DRIVING_FF", 1 / DRIVE_WHEEL_FREE_SPEED_RPS);
    public static final double DRIVING_MIN_OUTPUT_NORMALIZED = -1;
    public static final double DRIVING_MAX_OUTPUT_NORMALIZED = 1;

    public static final LoggedTunableNumber TURNING_P =
        new LoggedTunableNumber(
            "Drivetrain/constants/TURNING_P",
            1.0); // 1.0; // 1.0 might be a bit too much - reduce a bit if needed
    public static final LoggedTunableNumber TURNING_I =
        new LoggedTunableNumber("Drivetrain/constants/TURNING_I", 0);
    public static final LoggedTunableNumber TURNING_D =
        new LoggedTunableNumber("Drivetrain/constants/TURNING_D", 0);
    public static final LoggedTunableNumber TURNING_FF =
        new LoggedTunableNumber("Drivetrain/constants/TURNING_FF", 0);
    public static final double TURNING_MIN_OUTPUT_NORMALIZED = -1;
    public static final double TURNING_MAX_OUTPUT_NORMALIZED = 1;

    public static final IdleMode DRIVING_MOTOR_IDLE_MODE = IdleMode.kBrake;
    public static final IdleMode TURNING_MOTOR_IDLE_MODE = IdleMode.kBrake;

    public static final int DRIVING_MOTOR_CURRENT_LIMIT_AMPS = 40; // 50; // amps
    public static final int TURNING_MOTOR_CURRENT_LIMIT_AMPS = 20; // amps
  }

  public static final LoggedTunableNumber AUTOAIM_P = new LoggedTunableNumber("AUTOAIM_P", 1);
}
