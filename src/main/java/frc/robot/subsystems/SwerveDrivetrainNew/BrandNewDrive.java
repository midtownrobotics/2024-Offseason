package frc.robot.subsystems.SwerveDrivetrainNew;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.NeoDrivetrainConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Limelight.Limelight;
import frc.robot.subsystems.SwerveDrivetrainNew.SwerveDrivetrainIO.SwerveDrivetrainIO;
import frc.robot.subsystems.SwerveDrivetrainNew.SwerveDrivetrainIO.SwerveIOInputsAutoLogged;
import frc.robot.utils.LoggedTunableNumber;

public class BrandNewDrive extends SubsystemBase {

    public enum DriveState {
        MANUAL,
        FOLLOW_PATH,
        SPEAKER_AUTO_ALIGN,
        X,
        TUNING
    }

    private final SwerveDrivetrainIO m_swerveDrivetrainIO;
    private final SwerveIOInputsAutoLogged swerveIOInputs = new SwerveIOInputsAutoLogged();
    private final Limelight m_limelight;

    private DriveState state = DriveState.MANUAL;

    private final SwerveDrivePoseEstimator m_poseEstimator;

    private ChassisSpeeds driverChassisSpeeds;
    private ChassisSpeeds pathplannerChassisSpeeds;

    private PIDController autoAimPID = new PIDController(0.02, 0, 0.001);

    private boolean speedBoost;

    public BrandNewDrive(SwerveDrivetrainIO swerveDrivetrainIO, Limelight limelight) {
        m_swerveDrivetrainIO = swerveDrivetrainIO;
        m_limelight = limelight;

        m_poseEstimator = new SwerveDrivePoseEstimator(
			NeoDrivetrainConstants.DRIVE_KINEMATICS,
			Rotation2d.fromDegrees(m_swerveDrivetrainIO.getPigeonYaw()),
			m_swerveDrivetrainIO.getSwerveModulePositions(),
			new Pose2d()
		);

        m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));

        autoAimPID.setSetpoint(0);
    }

    @Override
    public void periodic() {
        m_swerveDrivetrainIO.updatePIDControllers();

        m_poseEstimator.update(
            Rotation2d.fromDegrees(m_swerveDrivetrainIO.getPigeonYaw()), 
            m_swerveDrivetrainIO.getSwerveModulePositions()
        );

        LimelightHelpers.PoseEstimate mt2 = m_limelight.getMegatagPose(getPose());

        if (mt2 != null) {
            m_poseEstimator.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
        }

        switch (state) {
            case SPEAKER_AUTO_ALIGN:
                if (m_limelight.isValidTarget(7)) {
                    m_swerveDrivetrainIO.drive(
                        driverChassisSpeeds.vxMetersPerSecond,
                        driverChassisSpeeds.vyMetersPerSecond,
                        -autoAimPID.calculate(m_limelight.getTx()), 
                        true, false, speedBoost);
                    break;
                }

                // Intentional Fall-through
            case TUNING:
            case MANUAL:
                m_swerveDrivetrainIO.drive(driverChassisSpeeds, true, speedBoost);
                break;
            case FOLLOW_PATH:
                m_swerveDrivetrainIO.drive(pathplannerChassisSpeeds, false, true);

            case X:
                m_swerveDrivetrainIO.drive(new SwerveModuleState[] {
                        new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                        new SwerveModuleState(0, Rotation2d.fromDegrees(45))
                    }
                );
                break;
        }

        // Original code has a calculateTurnAngleUsingPidController which seems to not do anything

        m_swerveDrivetrainIO.updateInputs(swerveIOInputs);
        swerveIOInputs.pose = getPose();
        Logger.processInputs("Drive", swerveIOInputs);
    }

    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        m_poseEstimator.resetPosition(
            Rotation2d.fromDegrees(m_swerveDrivetrainIO.getPigeonYaw()), 
            m_swerveDrivetrainIO.getSwerveModulePositions(), pose
        );
    }

    public void setBoost(boolean boost) {
        speedBoost = boost;
    }
    
}
