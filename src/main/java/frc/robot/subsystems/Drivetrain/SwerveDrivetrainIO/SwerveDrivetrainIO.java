package frc.robot.subsystems.Drivetrain.SwerveDrivetrainIO;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.NeoDrivetrainConstants;
import frc.robot.subsystems.Drivetrain.Drivetrain.DriveState;
import frc.robot.subsystems.Drivetrain.SwerveModuleIO.SwerveModuleIO;
import frc.robot.subsystems.Limelight.Limelight;

public interface SwerveDrivetrainIO {

  @AutoLog
  public class SwerveIOInputs {
    public Pose2d pose;
    public SwerveModuleState[] currentStates;
    public SwerveModuleState[] desiredStates;
    public double pigeonYaw;
    public DriveState state;
  }

  void updateInputs(SwerveIOInputs inputs);

  /**
   * reset heading to double value
   * @param heading to set
   */
  public void resetHeading(double heading);
  /**
   * reset heading to 0
   */
  public void resetHeading();

  /**
   * drive with chassis speeds
   * @param chassisSpeeds x, y, omega
   * @param fieldRelative true for field relative, false for robot relative
   * @param speedBoost true for fast, false for slow
   */
  default void drive(ChassisSpeeds chassisSpeeds, boolean fieldRelative, boolean speedBoost) {
    if (chassisSpeeds == null) {
      chassisSpeeds = new ChassisSpeeds();
    }

    drive(
        chassisSpeeds.vxMetersPerSecond,
        chassisSpeeds.vyMetersPerSecond,
        chassisSpeeds.omegaRadiansPerSecond,
        fieldRelative,
        false,
        speedBoost);
  }

  /**
   * drive x y omega
   * @param xSpeed
   * @param ySpeed
   * @param rot
   * @param fieldRelative true for field relative, false for robot relative
   * @param rateLimit ignore, alays set false
   * @param speedBoost true for fast, false for slow
   */
  void drive(
      double xSpeed,
      double ySpeed,
      double rot,
      boolean fieldRelative,
      boolean rateLimit,
      boolean speedBoost);

  /**
   * 
   * @param desiredStates
   */
  default void drive(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, NeoDrivetrainConstants.MAX_SPEED_METERS_PER_SECOND);
    getFrontLeftModule().setDesiredState(desiredStates[0]);
    getFrontRightModule().setDesiredState(desiredStates[1]);
    getRearLeftModule().setDesiredState(desiredStates[2]);
    getRearRightModule().setDesiredState(desiredStates[3]);
  }

  /**
   * get the rotation from the gyro
   * @return heading of the robot
   */
  public abstract double getPigeonYaw();

  // ODOMETRY DONE BY Drive class

  SwerveModuleIO getFrontLeftModule();

  SwerveModuleIO getFrontRightModule();

  SwerveModuleIO getRearLeftModule();

  SwerveModuleIO getRearRightModule();

  /**
   * get positions for all the swerve modules
   * @return SwerveModulePosition[]
   */
  default SwerveModulePosition[] getSwerveModulePositions() {
    return new SwerveModulePosition[] {
      getFrontLeftModule().getPosition(),
      getFrontRightModule().getPosition(),
      getRearLeftModule().getPosition(),
      getRearRightModule().getPosition()
    };
  }

  /**
   * get states for all the swerve modules
   * @return SwerveModuleState[]
   */
  default SwerveModuleState[] getSwerveModuleStates() {
    return new SwerveModuleState[] {
      getFrontLeftModule().getState(),
      getFrontRightModule().getState(),
      getRearLeftModule().getState(),
      getRearRightModule().getState()
    };
  }

  /**
   * get desired states for all the swerve modules
   * @return SwerveModuleState[]
   */
  default SwerveModuleState[] getSwerveModuleDesiredStates() {
    return new SwerveModuleState[] {
      getFrontLeftModule().getDesiredState(),
      getFrontRightModule().getDesiredState(),
      getRearLeftModule().getDesiredState(),
      getRearRightModule().getDesiredState()
    };
  }

  /**
   * get robot relative speeds
   * @return
   */
  default ChassisSpeeds getRobotRelativeSpeeds() {
    ChassisSpeeds output =
        Constants.NeoDrivetrainConstants.DRIVE_KINEMATICS.toChassisSpeeds(getSwerveModuleStates());
    return output;
  }

  /**
   * update PID controllers with new info
   */
  void updatePIDControllers();

  
  Pose2d getPose();

  
  void resetOdometry(Pose2d pose);

  /**
   * update odometry with new info
   */
  void updateOdometry();

  /**
   * update odometry with new info from limelight
   */
  void updateOdometryWithVision(Limelight limelight);
}
