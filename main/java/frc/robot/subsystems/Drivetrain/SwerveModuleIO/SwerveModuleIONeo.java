package frc.robot.subsystems.Drivetrain.SwerveModuleIO;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.NeoSwerveModuleConstants;
import frc.robot.utils.LoggedTunableNumber;

public class SwerveModuleIONeo implements SwerveModuleIO {
  private final SparkMax m_drivingSparkMax;
  private final SparkMax m_turningSparkMax;

  private final RelativeEncoder m_drivingEncoder;
  private final RelativeEncoder m_turningEncoder;
  private final CANcoder m_turningAbsoluteEncoder;

  private final SparkClosedLoopController m_drivingPIDController;
  private final SparkClosedLoopController m_turningPIDController;
  private double offset = 0;

  SparkMaxConfig m_turningMotorConfig = new SparkMaxConfig();
  SparkMaxConfig m_drivingMotorConfig = new SparkMaxConfig();

  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  public SwerveModuleIONeo(
      int drivingCANId,
      int turningCANId,
      int turningAnalogPort,
      double offset,
      boolean inverted
      ) {
    m_drivingSparkMax = new SparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSparkMax = new SparkMax(turningCANId, MotorType.kBrushless);
    
    this.offset = offset;

    m_drivingSparkMax.setInverted(inverted);

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    m_drivingEncoder = m_drivingSparkMax.getEncoder();
    m_turningEncoder = m_turningSparkMax.getEncoder();
    m_turningAbsoluteEncoder = new CANcoder(turningAnalogPort, "Sensors");
    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    // config.sensorCoefficient = 2 * Math.PI / 4096;
    // config.unitString = "rad";
    // config.sensorTimeBase = SensorTimeBase.PerSecond;
    // config.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
    config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    m_turningAbsoluteEncoder.getConfigurator().apply(config);
    // m_turningAbsoluteEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 100);

    m_drivingPIDController = m_drivingSparkMax.getClosedLoopController();
    m_turningPIDController = m_turningSparkMax.getClosedLoopController();
    
    // TODO: I COMMENTED OUT THE FOLLOWING LINES BECAUSE I DIDNT KNOW HOW TO REPLICATE THEM BUT IT MIGHT STILL WORK IDK -Gray
    // m_drivingPIDController.setFeedbackDevice(m_drivingEncoder);
    // m_turningPIDController.setFeedbackDevice(m_turningEncoder);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    m_drivingMotorConfig.encoder.positionConversionFactor(
        NeoSwerveModuleConstants.DRIVING_ENCODER_POSITION_FACTOR_METERS_PER_ROTATION);
        m_drivingMotorConfig.encoder.velocityConversionFactor(
        NeoSwerveModuleConstants.DRIVING_ENCODER_VELOCITY_FACTOR_METERS_PER_SECOND_PER_RPM);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve APIs.
    m_turningMotorConfig.encoder.positionConversionFactor(
        NeoSwerveModuleConstants.TURNING_ENCODER_POSITION_FACTOR_RADIANS_PER_ROTATION);
    m_turningMotorConfig.encoder.velocityConversionFactor(
        NeoSwerveModuleConstants.TURNING_ENCODER_VELOCITY_FACTOR_RADIANS_PER_SECOND_PER_RPM);

    // Invert the turning controller, since the output shaft rotates in the opposite direction of
    // the steering motor.
    m_turningSparkMax.setInverted(true);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    m_turningMotorConfig.closedLoop.positionWrappingEnabled(true);
    m_turningMotorConfig.closedLoop.positionWrappingMinInput(
        NeoSwerveModuleConstants.TURNING_ENCODER_POSITION_PID_MIN_INPUT_RADIANS);
        m_turningMotorConfig.closedLoop.positionWrappingMaxInput(
        NeoSwerveModuleConstants.TURNING_ENCODER_POSITION_PID_MAX_INPUT_RADIANS);

    // Set the PID gains for the driving motor.
    m_drivingMotorConfig.closedLoop.p(NeoSwerveModuleConstants.DRIVING_P.get());
    m_drivingMotorConfig.closedLoop.i(NeoSwerveModuleConstants.DRIVING_I.get());
    m_drivingMotorConfig.closedLoop.d(NeoSwerveModuleConstants.DRIVING_D.get());
    m_drivingMotorConfig.closedLoop.velocityFF(NeoSwerveModuleConstants.DRIVING_FF.get());
    m_drivingMotorConfig.closedLoop.outputRange(
        NeoSwerveModuleConstants.DRIVING_MIN_OUTPUT_NORMALIZED,
        NeoSwerveModuleConstants.DRIVING_MAX_OUTPUT_NORMALIZED);

    // Set the PID gains for the turning motor.
    m_turningMotorConfig.closedLoop.p(NeoSwerveModuleConstants.TURNING_P.get());
    m_turningMotorConfig.closedLoop.i(NeoSwerveModuleConstants.TURNING_I.get());
    m_turningMotorConfig.closedLoop.d(NeoSwerveModuleConstants.TURNING_D.get());
    m_turningMotorConfig.closedLoop.velocityFF(NeoSwerveModuleConstants.TURNING_FF.get());
    m_turningMotorConfig.closedLoop.outputRange(
        NeoSwerveModuleConstants.TURNING_MIN_OUTPUT_NORMALIZED,
        NeoSwerveModuleConstants.TURNING_MAX_OUTPUT_NORMALIZED);

    m_drivingMotorConfig.idleMode(IdleMode.kBrake);
    m_turningMotorConfig.idleMode(IdleMode.kBrake);
    m_drivingMotorConfig.smartCurrentLimit(
        NeoSwerveModuleConstants.DRIVING_MOTOR_CURRENT_LIMIT_AMPS);
    m_turningMotorConfig.smartCurrentLimit(
        NeoSwerveModuleConstants.TURNING_MOTOR_CURRENT_LIMIT_AMPS);

    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingEncoder.setPosition(0);

    resetEncoders();
  }

  @Override
  public void updateInputs(SwerveModuleIOInputs inputs) {
    inputs.turningCurrentAmps = m_turningSparkMax.getOutputCurrent();
    inputs.turningTempFahrenheit = m_turningSparkMax.getMotorTemperature();
    inputs.turningVelocityRPM = m_turningSparkMax.getEncoder().getVelocity();
    inputs.turningIsOn = Math.abs(m_turningSparkMax.getAppliedOutput()) > 0.01;
    inputs.turningVoltage =
        m_turningSparkMax.getAppliedOutput() * m_turningSparkMax.getBusVoltage();

    inputs.drivingCurrentAmps = m_drivingSparkMax.getOutputCurrent();
    inputs.drivingTempFahrenheit = m_drivingSparkMax.getMotorTemperature();
    inputs.drivingVelocityRPM = m_drivingSparkMax.getEncoder().getVelocity();
    inputs.drivingIsOn = Math.abs(m_drivingSparkMax.getAppliedOutput()) > 0.01;
    inputs.drivingVoltage =
        m_drivingSparkMax.getAppliedOutput() * m_drivingSparkMax.getBusVoltage();

    inputs.currentState = getState();
    inputs.desiredState = getDesiredState();
    inputs.offset = offset;
    inputs.turningEncoderPosition = getTurningEncoder().getPosition();
    inputs.turningAbsolutePosition = getAbsoluteTurningPosition();
  }

  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
    m_turningSparkMax.set(0);
    m_turningEncoder.setPosition(getAbsoluteTurningPosition() + offset);
  }

  public void calibrateVirtualPosition(double angle) {
    // if (this.offset != angle) {
    m_turningEncoder.setPosition(getAbsoluteTurningPosition() + angle);
    // }
    this.offset = angle;
  }

  public double getOffset() {
    return offset;
  }

  public RelativeEncoder getDrivingEncoder() {
    return m_drivingEncoder;
  }

  public RelativeEncoder getTurningEncoder() {
    return m_turningEncoder;
  }

  public CANcoder getTurningAbsoluteEncoder() {
    return m_turningAbsoluteEncoder;
  }

  public double getAbsoluteTurningPosition() {
    return m_turningAbsoluteEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
  }

  @Override
  public SwerveModuleState getDesiredState() {
    return m_desiredState;
  }

  @Override
  public void setDesiredState(SwerveModuleState desiredState) {
    m_desiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

    if (Math.abs(m_desiredState.speedMetersPerSecond) < 0.001
        && Math.abs(m_desiredState.angle.getRadians() - m_turningEncoder.getPosition())
            < Rotation2d.fromDegrees(1).getRadians()) {
      m_drivingSparkMax.set(0);
      m_turningSparkMax.set(0);
      return;
    }

    m_drivingPIDController.setReference(
      m_desiredState.speedMetersPerSecond, ControlType.kVelocity);
    m_turningPIDController.setReference(
        m_desiredState.angle.getRadians(), ControlType.kPosition);
  }

  @Override
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_drivingEncoder.getVelocity(), new Rotation2d(m_turningEncoder.getPosition()));
  }

  @Override
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(), new Rotation2d(m_turningEncoder.getPosition()));
  }

  public void updatePIDControllers() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          m_drivingMotorConfig.closedLoop.p(Constants.NeoSwerveModuleConstants.DRIVING_P.get());
          m_drivingMotorConfig.closedLoop.i(Constants.NeoSwerveModuleConstants.DRIVING_I.get());
          m_drivingMotorConfig.closedLoop.d(Constants.NeoSwerveModuleConstants.DRIVING_D.get());
          m_drivingMotorConfig.closedLoop.velocityFF(Constants.NeoSwerveModuleConstants.DRIVING_FF.get());
          m_drivingSparkMax.configure(m_drivingMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

          // m_drivingPIDController.setP(Constants.NeoSwerveModuleConstants.DRIVING_P.get());
          // m_drivingPIDController.setI(Constants.NeoSwerveModuleConstants.DRIVING_I.get());
          // m_drivingPIDController.setD(Constants.NeoSwerveModuleConstants.DRIVING_D.get());
          // m_drivingPIDController.setFF(Constants.NeoSwerveModuleConstants.DRIVING_FF.get());
        },
        Constants.NeoSwerveModuleConstants.DRIVING_P,
        Constants.NeoSwerveModuleConstants.DRIVING_I,
        Constants.NeoSwerveModuleConstants.DRIVING_D,
        Constants.NeoSwerveModuleConstants.DRIVING_FF);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          m_turningMotorConfig.closedLoop.p(Constants.NeoSwerveModuleConstants.TURNING_P.get());
          m_turningMotorConfig.closedLoop.i(Constants.NeoSwerveModuleConstants.TURNING_I.get());
          m_turningMotorConfig.closedLoop.d(Constants.NeoSwerveModuleConstants.TURNING_D.get());
          m_turningMotorConfig.closedLoop.velocityFF(Constants.NeoSwerveModuleConstants.TURNING_FF.get());
          m_drivingSparkMax.configure(m_turningMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        },
        Constants.NeoSwerveModuleConstants.TURNING_P,
        Constants.NeoSwerveModuleConstants.TURNING_I,
        Constants.NeoSwerveModuleConstants.TURNING_D,
        Constants.NeoSwerveModuleConstants.TURNING_FF);
  }
}
