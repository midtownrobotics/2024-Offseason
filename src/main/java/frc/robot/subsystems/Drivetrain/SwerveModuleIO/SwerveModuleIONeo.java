package frc.robot.subsystems.Drivetrain.SwerveModuleIO;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.NeoSwerveModuleConstants;

public class SwerveModuleIONeo implements SwerveModuleIO {
  private final SparkMax m_drivingSparkMax;
  private final SparkMax m_turningSparkMax;

  private final RelativeEncoder m_drivingEncoder;
  private final RelativeEncoder m_turningEncoder;
  private final CANcoder m_turningAbsoluteEncoder;

  private final SparkClosedLoopController m_drivingPIDController;
  private final SparkClosedLoopController m_turningPIDController;
  private double offset = 0;

  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  public SwerveModuleIONeo(
      int drivingCANId, int turningCANId, int turningAnalogPort, double offset, boolean inverted) {
    m_drivingSparkMax = new SparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSparkMax = new SparkMax(turningCANId, MotorType.kBrushless);
    this.offset = offset;

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    // m_drivingSparkMax.restoreFactoryDefaults();
    // m_turningSparkMax.restoreFactoryDefaults();
    // Invert the turning controller, since the output shaft rotates in the opposite direction of
    // the steering motor.
    SparkMaxConfig driveConfig = new SparkMaxConfig();
    SparkMaxConfig turningConfig = new SparkMaxConfig();

    driveConfig
        .closedLoop
        .pidf(
            NeoSwerveModuleConstants.DRIVING_P.get(),
            NeoSwerveModuleConstants.DRIVING_I.get(),
            NeoSwerveModuleConstants.DRIVING_D.get(),
            NeoSwerveModuleConstants.DRIVING_FF.get())
        .outputRange(
            NeoSwerveModuleConstants.DRIVING_MIN_OUTPUT_NORMALIZED,
            NeoSwerveModuleConstants.DRIVING_MAX_OUTPUT_NORMALIZED);

    driveConfig.inverted(inverted);
    driveConfig.idleMode(NeoSwerveModuleConstants.DRIVING_MOTOR_IDLE_MODE);
    driveConfig.encoder.positionConversionFactor(
        NeoSwerveModuleConstants.DRIVING_ENCODER_POSITION_FACTOR_METERS_PER_ROTATION);
    driveConfig.encoder.velocityConversionFactor(
        NeoSwerveModuleConstants.DRIVING_ENCODER_VELOCITY_FACTOR_METERS_PER_SECOND_PER_RPM);

    turningConfig.closedLoop.pidf(
        NeoSwerveModuleConstants.TURNING_P.get(),
        NeoSwerveModuleConstants.TURNING_I.get(),
        NeoSwerveModuleConstants.TURNING_D.get(),
        NeoSwerveModuleConstants.TURNING_FF.get());
    turningConfig.closedLoop.outputRange(
        NeoSwerveModuleConstants.TURNING_MIN_OUTPUT_NORMALIZED,
        NeoSwerveModuleConstants.TURNING_MAX_OUTPUT_NORMALIZED);

    turningConfig.closedLoop.positionWrappingEnabled(true);
    turningConfig.closedLoop.positionWrappingInputRange(
        NeoSwerveModuleConstants.TURNING_ENCODER_POSITION_PID_MIN_INPUT_RADIANS,
        NeoSwerveModuleConstants.TURNING_ENCODER_POSITION_PID_MAX_INPUT_RADIANS);

    driveConfig.smartCurrentLimit(NeoSwerveModuleConstants.DRIVING_MOTOR_CURRENT_LIMIT_AMPS);
    turningConfig.smartCurrentLimit(NeoSwerveModuleConstants.TURNING_MOTOR_CURRENT_LIMIT_AMPS);

    turningConfig.inverted(true);
    turningConfig.idleMode(NeoSwerveModuleConstants.TURNING_MOTOR_IDLE_MODE);

    turningConfig.encoder.positionConversionFactor(
        NeoSwerveModuleConstants.TURNING_ENCODER_POSITION_FACTOR_RADIANS_PER_ROTATION);
    turningConfig.encoder.velocityConversionFactor(
        NeoSwerveModuleConstants.TURNING_ENCODER_VELOCITY_FACTOR_RADIANS_PER_SECOND_PER_RPM);

    m_drivingSparkMax.configure(
        driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_turningSparkMax.configure(
        turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    m_drivingEncoder = m_drivingSparkMax.getEncoder();
    m_turningEncoder = m_turningSparkMax.getEncoder();

    m_turningAbsoluteEncoder = new CANcoder(turningAnalogPort, "Sensors");
    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    m_turningAbsoluteEncoder.getConfigurator().apply(config);
    // m_turningAbsoluteEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 100);

    m_drivingPIDController = m_drivingSparkMax.getClosedLoopController();
    m_turningPIDController = m_turningSparkMax.getClosedLoopController();

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.

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
    inputs.drivingEncoderPosition = getDrivingEncoder().getPosition();
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

  public double getDrivePositionRads() {
    return m_drivingEncoder.getPosition()
        / NeoSwerveModuleConstants.DRIVING_ENCODER_POSITION_FACTOR_METERS_PER_ROTATION
        * 2
        * Math.PI
        / NeoSwerveModuleConstants.DRIVING_MOTOR_REDUCTION;
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
    desiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));
    m_desiredState = desiredState;

    if (Math.abs(desiredState.speedMetersPerSecond) < 0.001
        && Math.abs(desiredState.angle.getRadians() - m_turningEncoder.getPosition())
            < Rotation2d.fromDegrees(1).getRadians()) {
      m_drivingSparkMax.set(0);
      m_turningSparkMax.set(0);

      return;
    }

    m_drivingPIDController.setReference(
        desiredState.speedMetersPerSecond, SparkMax.ControlType.kVelocity);
    m_turningPIDController.setReference(
        desiredState.angle.getRadians(), SparkMax.ControlType.kPosition);
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

  // public void updatePIDControllers() {
  //   LoggedTunableNumber.ifChanged(
  //       hashCode(),
  //       () -> {
  //         m_drivingPIDController.set(Constants.NeoSwerveModuleConstants.DRIVING_P.get());
  //         m_drivingPIDController.setI(Constants.NeoSwerveModuleConstants.DRIVING_I.get());
  //         m_drivingPIDController.setD(Constants.NeoSwerveModuleConstants.DRIVING_D.get());
  //         m_drivingPIDController.setFF(Constants.NeoSwerveModuleConstants.DRIVING_FF.get());
  //       },
  //       Constants.NeoSwerveModuleConstants.DRIVING_P,
  //       Constants.NeoSwerveModuleConstants.DRIVING_I,
  //       Constants.NeoSwerveModuleConstants.DRIVING_D,
  //       Constants.NeoSwerveModuleConstants.DRIVING_FF);

  //   LoggedTunableNumber.ifChanged(
  //       hashCode(),
  //       () -> {
  //         m_turningPIDController.setP(Constants.NeoSwerveModuleConstants.TURNING_P.get());
  //         m_turningPIDController.setI(Constants.NeoSwerveModuleConstants.TURNING_I.get());
  //         m_turningPIDController.setD(Constants.NeoSwerveModuleConstants.TURNING_D.get());
  //         m_turningPIDController.setFF(Constants.NeoSwerveModuleConstants.TURNING_FF.get());
  //       },
  //       Constants.NeoSwerveModuleConstants.TURNING_P,
  //       Constants.NeoSwerveModuleConstants.TURNING_I,
  //       Constants.NeoSwerveModuleConstants.TURNING_D,
  //       Constants.NeoSwerveModuleConstants.TURNING_FF);
  // }
}
