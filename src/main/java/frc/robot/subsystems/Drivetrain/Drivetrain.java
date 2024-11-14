package frc.robot.subsystems.Drivetrain;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import edu.wpi.first.math.MathUtil;
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
import frc.robot.utils.AllianceFlipUtil;
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
    TUNING,
    NOTE_AUTO_PICKUP
  }

  private final SwerveDrivetrainIO m_swerveDrivetrainIO;
  private final SwerveIOInputsAutoLogged swerveIOInputs = new SwerveIOInputsAutoLogged();
  private final Limelight m_limelight;
  private final Supplier<Boolean> beamBreakBrokenSupplier;

  private LoggedDashboardNumber vxMetersPerSecond = new LoggedDashboardNumber("Drivetrain/Tuning/vxMetersPerSecond");
  private LoggedDashboardNumber vyMetersPerSecond = new LoggedDashboardNumber("Drivetrain/Tuning/vyMetersPerSecond");
  private LoggedDashboardNumber omegaRadiansPerSecond = new LoggedDashboardNumber("Drivetrain/Tuning/vxMetersPerSecond");


  private DriveState state = DriveState.MANUAL;

  private ChassisSpeeds driverChassisSpeeds = new ChassisSpeeds(); // Robot Relative

  private ChassisSpeeds pathplannerChassisSpeeds = new ChassisSpeeds(); // Robot Relative

  private PIDController autoAimPID = new PIDController(0.2, 0, 0.01);

  private PIDController alignZeroPID = new PIDController(ShooterConstants.ALIGN_ZERO_P.get(), 0, 0);

  private boolean speedBoost;

  public Drivetrain(SwerveDrivetrainIO swerveDrivetrainIO, Limelight limelight, Supplier<Boolean> beamBreakBrokenSupplier) {
    m_swerveDrivetrainIO = swerveDrivetrainIO;
    m_limelight = limelight;
    this.beamBreakBrokenSupplier = beamBreakBrokenSupplier;

    autoAimPID.setSetpoint(0);
    noteAimPID.setSetpoint(0);

    alignZeroPID.enableContinuousInput(0, 360);
    alignZeroPID.setSetpoint(0);
  }

  private boolean lockedToNote = false;

  private PIDController autoPickUpX = new PIDController(0, 0, 0);
  private PIDController autoPickUpY = new PIDController(0.21, 0, 0);
  private PIDController noteAimPID = new PIDController(0.07, 0, 0);

  @Override
  public void periodic() {

    LoggedTunableNumber.ifChanged(hashCode(), () -> {
      alignZeroPID.setP(ShooterConstants.ALIGN_ZERO_P.get());
    }, ShooterConstants.ALIGN_ZERO_P);

    m_swerveDrivetrainIO.updatePIDControllers();

    m_swerveDrivetrainIO.updateOdometry();
    m_swerveDrivetrainIO.updateOdometryWithVision(m_limelight);

    if (!state.equals(DriveState.NOTE_AUTO_PICKUP)) {
      lockedToNote = false;
    }

    switch (state) {
      case NOTE_AUTO_PICKUP:
        if (!beamBreakBrokenSupplier.get()) {
          double rotAssist = noteAimPID.calculate(m_limelight.getTxFront());
          double xAssist = 0;
          // double yAssist = MathUtil.clamp(autoPickUpY.calculate(m_limelight.getTxFront()), -1, 1);
          double yAssist = 0;
          // double desiredX = 0.0;

          if (lockedToNote) {
            xAssist = 0.5;
          }

          if (m_limelight.isValidTargetNote() && Math.abs(m_limelight.getTxFront()) < 3) {
            lockedToNote = true;
            xAssist = 1;
          }



          ChassisSpeeds speeds = new ChassisSpeeds(xAssist+driverChassisSpeeds.vxMetersPerSecond, yAssist+driverChassisSpeeds.vyMetersPerSecond, rotAssist+driverChassisSpeeds.omegaRadiansPerSecond);

          m_swerveDrivetrainIO.chassisDrive(speeds);
        } else {
          m_swerveDrivetrainIO.chassisDrive(driverChassisSpeeds);
          lockedToNote = false;
        }
        break;
      case SPEAKER_AUTO_ALIGN:
        if (m_limelight.isValidTargetAprilTag(Tags.SPEAKER_CENTER.getId())) {
          double desiredOmega = autoAimPID.calculate(m_limelight.getTxBack());

          ChassisSpeeds speeds = new ChassisSpeeds(driverChassisSpeeds.vxMetersPerSecond, driverChassisSpeeds.vyMetersPerSecond, desiredOmega);

          m_swerveDrivetrainIO.chassisDrive(speeds);
          break;
        }

        // Intentional Fall-through - if Limelight does not detect target, we do manual driving
      case MANUAL:
        m_swerveDrivetrainIO.chassisDrive(driverChassisSpeeds);
        // m_swerveDrivetrainIO.drive(driveX, driveY, driveOmega, true, false, speedBoost);
        break;
      case FOLLOW_PATH_ALIGNED:
          if (m_limelight.isValidTargetAprilTag(Tags.SPEAKER_CENTER.getId())) {
            double desiredOmega = autoAimPID.calculate(m_limelight.getTxBack());

            ChassisSpeeds speeds = new ChassisSpeeds(pathplannerChassisSpeeds.vxMetersPerSecond, pathplannerChassisSpeeds.vyMetersPerSecond, desiredOmega);

            m_swerveDrivetrainIO.chassisDrive(speeds);
            break;
          }
      // INTENTIONAL FALL THROUGH
      case FOLLOW_PATH:
        m_swerveDrivetrainIO.chassisDrive(pathplannerChassisSpeeds);
        // m_swerveDrivetrainIO.drive(pathplannerChassisSpeeds, false, true);
        break;
      case X:
        // m_swerveDrivetrainIO.drive(4,
        //     0,
        //     0,
        //     false, false, speedBoost
        // )
        // ;
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

        ChassisSpeeds speeds = new ChassisSpeeds(driverChassisSpeeds.vxMetersPerSecond, driverChassisSpeeds.vyMetersPerSecond, desiredOmega);

        m_swerveDrivetrainIO.chassisDrive(speeds);
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

    if (AllianceFlipUtil.shouldFlip()) {
      speeds.vxMetersPerSecond = -speeds.vxMetersPerSecond;
      speeds.vyMetersPerSecond = -speeds.vyMetersPerSecond;
    }

    if ((Math.abs(speeds.vxMetersPerSecond) + Math.abs(speeds.vyMetersPerSecond)) > 1 && Math.abs(speeds.omegaRadiansPerSecond) > 0.25) {
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
