package frc.robot.subsystems.Drivetrain.SwerveDrivetrainIO;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import frc.robot.Constants;
import frc.robot.Constants.NeoDrivetrainConstants;
import frc.robot.subsystems.Drivetrain.SwerveModuleIO.SwerveModuleIOInputsAutoLogged;
import frc.robot.subsystems.Drivetrain.SwerveModuleIO.SwerveModuleIOSim;
import frc.robot.subsystems.Limelight.Limelight;

import org.littletonrobotics.junction.Logger;

public class SwerveDrivetrainIOSim implements SwerveDrivetrainIO {

  private static final SwerveModuleIOSim m_frontLeft = new SwerveModuleIOSim();
  private static final SwerveModuleIOSim m_frontRight = new SwerveModuleIOSim();
  private static final SwerveModuleIOSim m_rearLeft = new SwerveModuleIOSim();
  private static final SwerveModuleIOSim m_rearRight = new SwerveModuleIOSim();

  private final SwerveModuleIOInputsAutoLogged m_frontLeftInputs =
      new SwerveModuleIOInputsAutoLogged();
  private final SwerveModuleIOInputsAutoLogged m_frontRightInputs =
      new SwerveModuleIOInputsAutoLogged();
  private final SwerveModuleIOInputsAutoLogged m_rearLeftInputs =
      new SwerveModuleIOInputsAutoLogged();
  private final SwerveModuleIOInputsAutoLogged m_rearRightInputs =
      new SwerveModuleIOInputsAutoLogged();

  private final AnalogGyroSim m_gyro = new AnalogGyroSim(1);

  private final SwerveDrivePoseEstimator m_poseEstimator =
      new SwerveDrivePoseEstimator(
          NeoDrivetrainConstants.DRIVE_KINEMATICS,
          Rotation2d.fromRotations(getPigeonYaw()),
          getSwerveModulePositions(),
          new Pose2d());

  @Override
  public void updateInputs(SwerveIOInputs inputs) {
    m_frontLeft.updateInputs(m_frontLeftInputs);
    m_frontRight.updateInputs(m_frontRightInputs);
    m_rearLeft.updateInputs(m_rearLeftInputs);
    m_rearRight.updateInputs(m_rearRightInputs);
    Logger.processInputs("Drive/FrontLeft", m_frontLeftInputs);
    Logger.processInputs("Drive/FrontRight", m_frontRightInputs);
    Logger.processInputs("Drive/RearLeft", m_rearLeftInputs);
    Logger.processInputs("Drive/RearRight", m_rearRightInputs);

    inputs.pose = getPose();
    inputs.currentStates = getSwerveModuleStates();
    inputs.desiredStates = getSwerveModuleDesiredStates();
    inputs.pigeonYaw = getPigeonYaw();
  }

  @Override
  public void resetHeading() {
    m_gyro.setAngle(0);
  }

  @Override
  public void resetHeading(double heading) {
    m_gyro.setAngle(heading);
  }

  @Override
  public void chassisDrive(ChassisSpeeds speeds) {
    SwerveModuleState[] swerveModuleStates = NeoDrivetrainConstants.DRIVE_KINEMATICS.toSwerveModuleStates(speeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.NeoDrivetrainConstants.MAX_SPEED_METERS_PER_SECOND_BOOSTED);

    drive(swerveModuleStates);
  }

  @Override
  public double getPigeonYaw() {
    return m_gyro.getAngle();
  }

  @Override
  public SwerveModuleIOSim getFrontLeftModule() {
    return m_frontLeft;
  }

  @Override
  public SwerveModuleIOSim getFrontRightModule() {
    return m_frontRight;
  }

  @Override
  public SwerveModuleIOSim getRearLeftModule() {
    return m_rearLeft;
  }

  @Override
  public SwerveModuleIOSim getRearRightModule() {
    return m_rearRight;
  }

  @Override
  public void updatePIDControllers() {}

  @Override
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  @Override
  public void resetOdometry(Pose2d pose) {
    // resetHeading(AllianceFlipUtil.apply(pose.getRotation()).getDegrees());
    m_poseEstimator.resetPosition(
        Rotation2d.fromRotations(getPigeonYaw()), getSwerveModulePositions(), pose);
  }

  @Override
  public void updateOdometry() {
    ChassisSpeeds robotSpeeds =
        Constants.NeoDrivetrainConstants.DRIVE_KINEMATICS.toChassisSpeeds(getSwerveModuleStates());
    double angularVelocity = robotSpeeds.omegaRadiansPerSecond;

    double deltaAngle = angularVelocity * 0.02;

    double newAngle = m_gyro.getAngle() + deltaAngle;

    Logger.recordOutput("Gyro/robotSpeeds", robotSpeeds);
    Logger.recordOutput("Gyro/angularVel", angularVelocity);
    Logger.recordOutput("Gyro/deltaAngle", deltaAngle);
    Logger.recordOutput("Gyro/oldAngle", newAngle - deltaAngle);
    Logger.recordOutput("Gyro/newAngle", newAngle);

    m_gyro.setAngle(newAngle);

    m_poseEstimator.update(Rotation2d.fromRotations(getPigeonYaw()), getSwerveModulePositions());
  }

  @Override
  public void updateOdometryWithVision(Limelight limelight) {}
}
