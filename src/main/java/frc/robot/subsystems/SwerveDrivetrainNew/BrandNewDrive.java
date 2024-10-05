package frc.robot.subsystems.SwerveDrivetrainNew;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.NeoDrivetrainConstants;
import frc.robot.subsystems.Limelight.Limelight;
import frc.robot.subsystems.SwerveDrivetrainNew.SwerveDrivetrainIO.SwerveDrivetrainIO;

public class BrandNewDrive extends SubsystemBase {

    public enum DriveState {
        MANUAL,
        FOLLOW_PATH,
        SPEAKER_AUTO_ALIGN,
        X,
        TUNING
    }

    private final SwerveDrivetrainIO m_swerveDrivetrainIO;
    private final Limelight m_limelight;

    private final SwerveDrivePoseEstimator m_poseEstimator;

    public BrandNewDrive(SwerveDrivetrainIO swerveDrivetrainIO, Limelight limelight) {
        m_swerveDrivetrainIO = swerveDrivetrainIO;
        m_limelight = limelight;

        m_poseEstimator = new SwerveDrivePoseEstimator(
			NeoDrivetrainConstants.DRIVE_KINEMATICS,
			Rotation2d.fromDegrees(m_swerveDrivetrainIO.getPigeonYaw()),
			m_swerveDrivetrainIO.geSwerveModulePositions(),
			new Pose2d()
		);
    }

    @Override
    public void periodic() {
        m_poseEstimator.update(
            Rotation2d.fromDegrees(m_swerveDrivetrainIO.getPigeonYaw()), 
            m_swerveDrivetrainIO.geSwerveModulePositions()
        );

        m_swerveDrivetrainIO.updatePIDControllers();

        // Original code has a calculateTurnAngleUsingPidController which seems to not do anything
    }
    
}
