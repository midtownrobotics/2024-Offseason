package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain.Drivetrain;
import frc.robot.utils.LoggedTunablePIDController;
import org.littletonrobotics.junction.Logger;

public class DriveToPoint extends Command {
  public static final double kMaxLinearSpeed = 2.0; // meters per second
  public static final double kMaxLinearAcceleration = 3; // meters per second squared
  public static final double kTrackWidthX = Units.inchesToMeters(15.25);
  public static final double kTrackWidthY = Units.inchesToMeters(16.25);
  public static final double kDriveBaseRadius = Math.hypot(kTrackWidthX / 2.0, kTrackWidthY / 2.0);
  public static final double kMaxAngularSpeed = 0.9;
  public static final double kMaxAngularAcceleration = 0.9;

  private LoggedTunablePIDController m_thetaController =
      new LoggedTunablePIDController("/RohanPoint/Theta", 1, 0, .1);

  private Drivetrain m_drive;
  private Pose2d m_targetPose;

  private ProfiledPIDController m_driveController =
      new ProfiledPIDController(
          1, 0.0, 0.3, new TrapezoidProfile.Constraints(kMaxLinearSpeed, kMaxLinearAcceleration));

  private ProfiledPIDController m_headingController =
      new ProfiledPIDController(
          2.0,
          0.0,
          0.0,
          new TrapezoidProfile.Constraints(kMaxAngularSpeed, kMaxAngularAcceleration));

  private double m_ffMinRadius = 0.5, m_ffMaxRadius = 1.1;

  public DriveToPoint(Drivetrain drive, Pose2d targetPose) {
    m_drive = drive;
    m_targetPose = targetPose;
    addRequirements(m_drive);

    m_headingController.enableContinuousInput(-Math.PI, Math.PI);

    m_driveController.setTolerance(Units.inchesToMeters(0.5));
    m_headingController.setTolerance(Units.degreesToRadians(1));

    m_thetaController.getController().enableContinuousInput(-Math.PI, Math.PI);
    m_thetaController.getController().setTolerance(Units.degreesToRadians(0.5));
  }

  @Override
  public void initialize() {
    Pose2d currentPose = m_drive.getPose();
    ChassisSpeeds fieldRelative =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            m_drive.getRobotRelativeSpeeds(), currentPose.getRotation());
    m_driveController.reset(
        currentPose.getTranslation().getDistance(m_targetPose.getTranslation()),
        Math.min(
            0.0,
            -new Translation2d(fieldRelative.vxMetersPerSecond, fieldRelative.vyMetersPerSecond)
                .rotateBy(
                    m_targetPose
                        .getTranslation()
                        .minus(currentPose.getTranslation())
                        .getAngle()
                        .unaryMinus())
                .getX()));
    m_headingController.reset(
        currentPose.getRotation().getRadians(), fieldRelative.omegaRadiansPerSecond);

    m_thetaController.getController().reset();
  }

  @Override
  public void execute() {
    Pose2d currentPose = m_drive.getPose();

    double currentDistance =
        currentPose.getTranslation().getDistance(m_targetPose.getTranslation());
    double ffScaler =
        MathUtil.clamp(
            (currentDistance - m_ffMinRadius) / (m_ffMaxRadius - m_ffMinRadius), 0.0, 1.0);
    double driveVelocityScalar =
        m_driveController.getSetpoint().velocity * ffScaler
            + m_driveController.calculate(currentDistance, 0.0);
    if (currentDistance < m_driveController.getPositionTolerance()) {
      driveVelocityScalar = 0.0;
    }

    double thetaVelocity = 0;
    if (!m_thetaController.getController().atSetpoint() || true) {
      thetaVelocity =
          m_thetaController
              .getController()
              .calculate(
                  currentPose.getRotation().getRadians(), m_targetPose.getRotation().getRadians());
    }

    double headingError = currentPose.getRotation().minus(m_targetPose.getRotation()).getRadians();
    // double headingVelocity = m_headingController.getSetpoint().velocity * ffScaler
    //     + m_headingController.calculate(
    //         currentPose.getRotation().getRadians(), m_targetPose.getRotation().getRadians());

    // if (Math.abs(headingError) < m_headingController.getPositionTolerance() || false) {
    //   headingVelocity = 0.0;
    // }

    // evil math
    // blame 254 for making this because i dont fully understand it
    Translation2d driveVelocity =
        new Pose2d(
                0.0,
                0.0,
                currentPose.getTranslation().minus(m_targetPose.getTranslation()).getAngle())
            .transformBy(new Transform2d(driveVelocityScalar, 0.0, new Rotation2d()))
            .getTranslation();
    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, currentPose.getRotation());
    m_drive.setDriveToPointDesired(speeds);

    Logger.recordOutput("DriveToPoint/TargetPose", m_targetPose);
    Logger.recordOutput("DriveToPoint/DriveDistance", currentDistance);
    Logger.recordOutput("DriveToPoint/HeadingError", headingError);

    Logger.recordOutput("DriveToPoint/DriveVelocityScalar", driveVelocityScalar);
    Logger.recordOutput("DriveToPoint/HeadingVelocity", thetaVelocity);
    Logger.recordOutput("DriveToPoint/DriveVelocityX", driveVelocity.getX());
    Logger.recordOutput("DriveToPoint/DriveVelocityY", driveVelocity.getY());

    Logger.recordOutput("DriveToPoint/DriveSpeeds", speeds);
  }

  @Override
  public boolean isFinished() {
    return m_driveController.atGoal() && m_headingController.atGoal();
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.setDriveToPointDesired(new ChassisSpeeds());
  }
}
