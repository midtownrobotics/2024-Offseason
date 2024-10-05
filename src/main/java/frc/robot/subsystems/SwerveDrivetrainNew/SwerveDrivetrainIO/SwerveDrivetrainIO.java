package frc.robot.subsystems.SwerveDrivetrainNew.SwerveDrivetrainIO;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Constants.NeoDrivetrainConstants;
import frc.robot.subsystems.SwerveDrivetrainNew.SwerveModuleIO.SwerveModuleIO;
import frc.robot.subsystems.drivetrain.NeoSwerveDrive.NeoSwerveModule;

public abstract class SwerveDrivetrainIO {

    private final SwerveModuleIO m_frontLeft;
    private final SwerveModuleIO m_frontRight;
    private final SwerveModuleIO m_rearLeft;
    private final SwerveModuleIO m_rearRight;

    protected SwerveDrivetrainIO(SwerveModuleIO frontLeft, SwerveModuleIO frontRight, SwerveModuleIO rearLeft, SwerveModuleIO rearRight) {
        this.m_frontLeft = frontLeft;
        this.m_frontRight = frontRight;
        this.m_rearLeft = rearLeft;
        this.m_rearRight = rearRight;

        resetHeading();
    }

    public abstract void resetHeading();

    public abstract double getPigeonYaw();

    // ODOMETRY DONE BY Drive class
    
    protected SwerveModuleIO getFrontLeftModule() {
		return m_frontLeft;
	}

	protected SwerveModuleIO getFrontRightModule() {
		return m_frontRight;
	}

	protected SwerveModuleIO getRearLeftModule() {
		return m_rearLeft;
	}

	protected SwerveModuleIO getRearRightModule() {
		return m_rearRight;
	}

    public SwerveModulePosition[] geSwerveModulePositions() {
        return new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        };
    }

    public void updatePIDControllers() {
        m_frontLeft.updatePIDControllers();
        m_frontRight.updatePIDControllers();
        m_rearLeft.updatePIDControllers();
        m_rearRight.updatePIDControllers();
    }
}
