package frc.robot.subsystems.Drivetrain;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Drivetrain.SwerveDrivetrainIO.SwerveDrivetrainIO;
import frc.robot.subsystems.Drivetrain.SwerveDrivetrainIO.SwerveIOInputsAutoLogged;
import frc.robot.subsystems.Limelight.Limelight;
import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.ApriltagHelper.Tags;

public class Drivetrain extends SubsystemBase {

  public enum DriveState {
    MANUAL,
    FOLLOW_PATH,
    FOLLOW_PATH_ALIGNED,
    SPEAKER_AUTO_ALIGN,
    ALIGN_ZERO,
    X,
    TUNING
  }

  private final SwerveDrivetrainIO m_swerveDrivetrainIO;
  private final SwerveIOInputsAutoLogged swerveIOInputs = new SwerveIOInputsAutoLogged();
  private final Limelight m_limelight;

  private LoggedDashboardNumber vxMetersPerSecond = new LoggedDashboardNumber("Drivetrain/Tuning/vxMetersPerSecond");
  private LoggedDashboardNumber vyMetersPerSecond = new LoggedDashboardNumber("Drivetrain/Tuning/vyMetersPerSecond");
  private LoggedDashboardNumber omegaRadiansPerSecond = new LoggedDashboardNumber("Drivetrain/Tuning/vxMetersPerSecond");


  private DriveState state = DriveState.MANUAL;

  private ChassisSpeeds driverChassisSpeeds = new ChassisSpeeds(); // Robot Relative
  // private double driveX;
  // private double driveY;
  // private double driveOmega;
  private ChassisSpeeds pathplannerChassisSpeeds = new ChassisSpeeds(); // Robot Relative

  private PIDController autoAimPID = new PIDController(0.02, 0, 0.001);

  private PIDController alignZeroPID = new PIDController(ShooterConstants.ALIGN_ZERO_P.get(), 0, 0);

  private boolean speedBoost;

  public Drivetrain(SwerveDrivetrainIO swerveDrivetrainIO, Limelight limelight) {
    m_swerveDrivetrainIO = swerveDrivetrainIO;
    m_limelight = limelight;

    autoAimPID.setSetpoint(0);

    alignZeroPID.enableContinuousInput(0, 360);
    alignZeroPID.setSetpoint(0);
  }

  @Override
  public void periodic() {

    LoggedTunableNumber.ifChanged(hashCode(), () -> {
      alignZeroPID.setP(ShooterConstants.ALIGN_ZERO_P.get());
    }, ShooterConstants.ALIGN_ZERO_P);

    m_swerveDrivetrainIO.updatePIDControllers();

    m_swerveDrivetrainIO.updateOdometry();
    m_swerveDrivetrainIO.updateOdometryWithVision(m_limelight);

    switch (state) {
      case SPEAKER_AUTO_ALIGN:
        if (m_limelight.isValidTarget(Tags.SPEAKER_CENTER.getId())) {
          double desiredOmega = autoAimPID.calculate(m_limelight.getTx());

          // if (DriverStation.getAlliance().get() == Alliance.Blue) {
          //   desiredOmega *= -1;
          // }

          m_swerveDrivetrainIO.drive(
              driverChassisSpeeds.vxMetersPerSecond,
              driverChassisSpeeds.vyMetersPerSecond,
              desiredOmega,
              false,
              false,
              speedBoost);
          break;
        }

        // Intentional Fall-through - if Limelight does not detect target, we do manual driving
      case MANUAL:
        m_swerveDrivetrainIO.drive(driverChassisSpeeds, false, speedBoost);
        // m_swerveDrivetrainIO.drive(driveX, driveY, driveOmega, true, false, speedBoost);
        break;
      case FOLLOW_PATH_ALIGNED:
          if (m_limelight.isValidTarget(Tags.SPEAKER_CENTER.getId())) {
            double desiredOmega = autoAimPID.calculate(m_limelight.getTx());

            // if (DriverStation.getAlliance().get() == Alliance.Blue) {
            //   desiredOmega *= -1;
            // }

            m_swerveDrivetrainIO.drive(
                pathplannerChassisSpeeds.vxMetersPerSecond,
                pathplannerChassisSpeeds.vyMetersPerSecond,
                desiredOmega,
                false,
                false,
                speedBoost);
            break;
          }
      // INTENTIONAL FALL THROUGH
      case FOLLOW_PATH:
        m_swerveDrivetrainIO.drive(
          pathplannerChassisSpeeds.vxMetersPerSecond,
          pathplannerChassisSpeeds.vyMetersPerSecond,
          0, // PathPlanner banned from rotating robot
          false,
          false,
          speedBoost
        );
        // m_swerveDrivetrainIO.drive(pathplannerChassisSpeeds, false, true);
        break;
      case X:
        // m_swerveDrivetrainIO.drive(4,
        //     0,
        //     0,
        //     false, false, speedBoost
        // );
        m_swerveDrivetrainIO.drive(
            new SwerveModuleState[] {
              new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
              new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
              new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
              new SwerveModuleState(0, Rotation2d.fromDegrees(45))
            });
        break;
      case ALIGN_ZERO:
        double desiredOmega = alignZeroPID.calculate(getAngle());

        m_swerveDrivetrainIO.drive(
            driverChassisSpeeds.vxMetersPerSecond,
            driverChassisSpeeds.vyMetersPerSecond,
            desiredOmega,
            false,
            false,
            speedBoost);
      break;
      case TUNING:
        setDriverDesired(new ChassisSpeeds(vxMetersPerSecond.get(), vyMetersPerSecond.get(), omegaRadiansPerSecond.get()));
    }

    // Logger.recordOutput("Drive/DrivetrainState", state.toString());

    // Original code has a calculateTurnAngleUsingPidController which seems to not do anything

    m_swerveDrivetrainIO.updateInputs(swerveIOInputs);
    swerveIOInputs.state = state;
    // swerveIOInputs.pose = getPose();
    Logger.processInputs("Drive", swerveIOInputs);
  }

  public void setState(DriveState state) {
    this.state = state;
  }

  public void setBoost(boolean boost) {
    speedBoost = boost;
  }

  // public void setDriverDesired(double driveX, double driveY, double driveOmega) {
  //     this.driveX = driveX;
  //     this.driveY = driveY;
  //     this.driveOmega = driveOmega;
  // }

  public void setDriverDesired(ChassisSpeeds speeds) {
    boolean discretizing = false;
    if ((Math.abs(speeds.vxMetersPerSecond) + Math.abs(speeds.vyMetersPerSecond)) > 1
        && Math.abs(speeds.omegaRadiansPerSecond) > 0.25) {
      speeds = ChassisSpeeds.discretize(speeds, 0.02);
      discretizing = true;
    }
    Logger.recordOutput("Drive/Discretizing", discretizing);
    this.driverChassisSpeeds = speeds;
  }

  public void setPathPlannerDesired(ChassisSpeeds speeds) {
    pathplannerChassisSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    Logger.recordOutput("Drive/PathPlannerSpeed", pathplannerChassisSpeeds);
  }

  public double getAngle() {
    return m_swerveDrivetrainIO.getPigeonYaw();
  }

  public void resetHeading() {
    m_swerveDrivetrainIO.resetHeading();
  }

  public void resetHeading(double heading) {
    m_swerveDrivetrainIO.resetHeading(heading);
  }

  public Pose2d getPose() {
    return m_swerveDrivetrainIO.getPose();
  }

  public void resetOdometry(Pose2d pose) {
    m_swerveDrivetrainIO.resetOdometry(pose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    ChassisSpeeds output =
        Constants.NeoDrivetrainConstants.DRIVE_KINEMATICS.toChassisSpeeds(
            m_swerveDrivetrainIO.getSwerveModuleStates());
    return output;
  }

  public ChassisSpeeds getRobotRelativeSpeedsNoOmega() {
    ChassisSpeeds speeds = getRobotRelativeSpeeds();
    return new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, 0);
  }
}
