package frc.robot.subsystems.SwerveDrivetrainNew.SwerveDrivetrainIO;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import frc.robot.Constants.NeoDrivetrainConstants;
import frc.robot.Ports;
import frc.robot.subsystems.SwerveDrivetrainNew.SwerveModuleIO.SwerveModuleIO.SwerveModuleIOInputs;
import frc.robot.subsystems.SwerveDrivetrainNew.SwerveModuleIO.SwerveModuleIOInputsAutoLogged;
import frc.robot.subsystems.SwerveDrivetrainNew.SwerveModuleIO.SwerveModuleIONeo;
import frc.robot.utils.NeoSwerveUtils;

public class SwerveDrivetrainIONeo extends SwerveDrivetrainIO{

    private static final double FRONT_LEFT_VIRTUAL_OFFSET_RADIANS = -3.03+Math.PI; // adjust as needed so that virtual (turn) position of wheel is zero when straight
	private static final double FRONT_RIGHT_VIRTUAL_OFFSET_RADIANS = 2.69; // adjust as needed so that virtual (turn) position of wheel is zero when straight
	private static final double REAR_LEFT_VIRTUAL_OFFSET_RADIANS = -2.33; // adjust as needed so that virtual (turn) position of wheel is zero when straight
	private static final double REAR_RIGHT_VIRTUAL_OFFSET_RADIANS = Math.PI/2+Math.PI/16-Math.PI; // adjust as needed so that virtual (turn) position of wheel is zero when straight
    private static final int GYRO_ORIENTATION = 1; // might be able to merge with kGyroReversed

    private static final double FIELD_LENGTH_INCHES = 54*12+1; // 54ft 1in
	private static final double FIELD_WIDTH_INCHES = 26*12+7; // 26ft 7in

    /** TURN SETTINGS */
	// NOTE: it might make sense to decrease the PID controller period below 0.02 sec (which is the period used by the main loop)
	private static final double TURN_PID_CONTROLLER_PERIOD_SECONDS = .02; // 0.01 sec = 10 ms 	
	
	private static final double MIN_TURN_PCT_OUTPUT = 0.1; // 0.1;
	private static final double MAX_TURN_PCT_OUTPUT = 0.4; // 0.4;
	
	private static final double TURN_PROPORTIONAL_GAIN = 0.001; // 0.01;
	private static final double TURN_INTEGRAL_GAIN = 0.0;
	private static final double TURN_DERIVATIVE_GAIN = 0.0; // 0.0001
	
	private static final int DEGREE_THRESHOLD = 10; // 3;
	
	private final static int TURN_ON_TARGET_MINIMUM_COUNT = 10; // number of times/iterations we need to be on target to really be on target
	/** END TURN SETTINGS */

    // Create SwerveModules
	private final SwerveModuleIONeo m_frontLeft /* #2 */;
	private final SwerveModuleIONeo m_frontRight /* #1 */;
	private final SwerveModuleIONeo m_rearLeft /* #3 */;
	private final SwerveModuleIONeo m_rearRight /* #4 */;

    private final SwerveModuleIOInputs frontLeftIOInputs = new SwerveModuleIOInputsAutoLogged();
    private final SwerveModuleIOInputs frontRightIOInputs = new SwerveModuleIOInputsAutoLogged();
    private final SwerveModuleIOInputs rearLeftIOInputs = new SwerveModuleIOInputsAutoLogged();
    private final SwerveModuleIOInputs rearRightIOInputs = new SwerveModuleIOInputsAutoLogged();

    private final WPI_Pigeon2 m_pigeon = new WPI_Pigeon2(5, "Sensors");

    // Slew rate filter variables for controlling lateral acceleration
    private double m_currentRotation = 0.0;
    private double m_currentTranslationDir = 0.0;
    private double m_currentTranslationMag = 0.0;

    private SlewRateLimiter m_magLimiter = new SlewRateLimiter(NeoDrivetrainConstants.MAGNITUDE_SLEW_RATE);
    private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(NeoDrivetrainConstants.ROTATIONAL_SLEW_RATE);
    private double m_prevTime = WPIUtilJNI.now() * 1e-6;

    // other variables
	private boolean isTurning;  // indicates that the drivetrain is turning using the PID controller hereunder

	private int onTargetCountTurning; // counter indicating how many times/iterations we were on target

	private PIDController m_turnPidController; // the PID controller used to turn

    public SwerveDrivetrainIONeo() {
        super(
            new SwerveModuleIONeo(
                Ports.NeoDrive.FRONT_LEFT_DRIVING,
                Ports.NeoDrive.FRONT_LEFT_TURNING,
                Ports.NeoDrive.FRONT_LEFT_TURNING_ABSOLUTE_ENCODER,
                -0.317, false, "FrontLeft"
            ),
            new SwerveModuleIONeo(
                Ports.NeoDrive.FRONT_RIGHT_DRIVING,
                Ports.NeoDrive.FRONT_RIGHT_TURNING,
                Ports.NeoDrive.FRONT_RIGHT_TURNING_ABSOLUTE_ENCODER,
                0.86, true, "FrontRight"
            ),
            new SwerveModuleIONeo(
                Ports.NeoDrive.REAR_LEFT_DRIVING,
                Ports.NeoDrive.REAR_LEFT_TURNING,
                Ports.NeoDrive.REAR_LEFT_TURNING_ABSOLUTE_ENCODER,
                -0.9, true, "RearLeft"
            ),
            new SwerveModuleIONeo(
                Ports.NeoDrive.REAR_RIGHT_DRIVING,
                Ports.NeoDrive.REAR_RIGHT_TURNING,
                Ports.NeoDrive.REAR_RIGHT_TURNING_ABSOLUTE_ENCODER,
                -0.33, false, "RearRight"
            )
        );

        m_frontLeft = (SwerveModuleIONeo) super.getFrontLeftModule();
        m_frontRight = (SwerveModuleIONeo) super.getFrontRightModule();
        m_rearLeft = (SwerveModuleIONeo) super.getRearLeftModule();
        m_rearRight = (SwerveModuleIONeo) super.getRearRightModule();

        m_turnPidController = new PIDController(TURN_PROPORTIONAL_GAIN, TURN_INTEGRAL_GAIN, TURN_DERIVATIVE_GAIN);
        m_turnPidController.enableContinuousInput(-180, 180);
        m_turnPidController.setTolerance(DEGREE_THRESHOLD);
    }

    @Override
    public void updateInputs(SwerveIOInputs inputs) {
        m_frontLeft.updateInputs(inputs.frontLeft);
        m_frontRight.updateInputs(inputs.frontRight);
        m_rearLeft.updateInputs(inputs.rearLeft);
        m_rearRight.updateInputs(inputs.rearRight);
    }

    @Override
    public void resetHeading() {
        m_pigeon.setYaw(0);
    }

    @Override
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit, boolean speedBoost) {
		
		double xSpeedCommanded;
		double ySpeedCommanded;

		if (rateLimit) {
			// Convert XY to polar for rate limiting
			double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
			double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

			// Calculate the direction slew rate based on an estimate of the lateral acceleration
			double directionSlewRate;

			if (m_currentTranslationMag != 0.0) {
				directionSlewRate = Math.abs(NeoDrivetrainConstants.DIRECTION_SLEW_RATE / m_currentTranslationMag);
			} else {
				directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
			}
			

			double currentTime = WPIUtilJNI.now() * 1e-6;
			double elapsedTime = currentTime - m_prevTime;
			double angleDif = NeoSwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);

			if (angleDif < 0.45*Math.PI) {
				m_currentTranslationDir = NeoSwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
				m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
			}
			else if (angleDif > 0.85*Math.PI) {
				if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
					// keep currentTranslationDir unchanged
					m_currentTranslationMag = m_magLimiter.calculate(0.0);
				}
				else {
					m_currentTranslationDir = NeoSwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
					m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
				}
			}
			else {
				m_currentTranslationDir = NeoSwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
				m_currentTranslationMag = m_magLimiter.calculate(0.0);
			}

			m_prevTime = currentTime;
			
			xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
			ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
			m_currentRotation = m_rotLimiter.calculate(rot);

		} else {
			xSpeedCommanded = xSpeed;
			ySpeedCommanded = ySpeed;
			m_currentRotation = rot;
		}

		// Convert the commanded speeds into the correct units for the drivetrain
		double maxSpeed;

		if (speedBoost) {
			maxSpeed = NeoDrivetrainConstants.MAX_SPEED_METERS_PER_SECOND_BOOSTED;
		} else {
			maxSpeed = NeoDrivetrainConstants.MAX_SPEED_METERS_PER_SECOND;
		}

		double xSpeedDelivered = xSpeedCommanded * maxSpeed;
		double ySpeedDelivered = ySpeedCommanded * maxSpeed;

		// Logger.recordOutput("NeoSwerve/xSpeedDesired", xSpeedDelivered);
		// Logger.recordOutput("NeoSwerve/ySpeedDesired", ySpeedDelivered);

		double rotDelivered = m_currentRotation * NeoDrivetrainConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;

		// Logger.recordOutput("NeoSwerve/rotationDesired", rotDelivered);

		SwerveModuleState[] swerveModuleStates = NeoDrivetrainConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
			fieldRelative
				? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(getPigeonYaw()))
				: new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));

		SwerveDriveKinematics.desaturateWheelSpeeds(
			swerveModuleStates, maxSpeed);

		m_frontLeft.setDesiredState(swerveModuleStates[0]);
		m_frontRight.setDesiredState(swerveModuleStates[1]);
		m_rearLeft.setDesiredState(swerveModuleStates[2]);
		m_rearRight.setDesiredState(swerveModuleStates[3]);
	}


    @Override
    public double getPigeonYaw() {
        return m_pigeon.getYaw() * GYRO_ORIENTATION;
    }

    @Override
    protected SwerveModuleIONeo getFrontLeftModule() {
		return m_frontLeft;
	}

    @Override
	protected SwerveModuleIONeo getFrontRightModule() {
		return m_frontRight;
	}

    @Override
	protected SwerveModuleIONeo getRearLeftModule() {
		return m_rearLeft;
	}

    @Override
	protected SwerveModuleIONeo getRearRightModule() {
		return m_rearRight;
	}


}
