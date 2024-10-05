package frc.robot.subsystems.SwerveDrivetrainNew.SwerveDrivetrainIO;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.NeoDrivetrainConstants;
import frc.robot.subsystems.SwerveDrivetrainNew.SwerveModuleIO.SwerveModuleIO;
import frc.robot.subsystems.SwerveDrivetrainNew.SwerveModuleIO.SwerveModuleIO.SwerveModuleIOInputs;

public abstract class SwerveDrivetrainIO {

    @AutoLog
    public class SwerveIOInputs {
        public SwerveModuleIOInputs frontLeft = new SwerveModuleIOInputs();
        public SwerveModuleIOInputs frontRight = new SwerveModuleIOInputs();
        public SwerveModuleIOInputs rearLeft = new SwerveModuleIOInputs();
        public SwerveModuleIOInputs rearRight = new SwerveModuleIOInputs();
        public Pose2d pose;
    }

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

    public abstract void updateInputs(SwerveIOInputs inputs);

    public abstract void resetHeading();

    public void drive(ChassisSpeeds chassisSpeeds, boolean fieldRelative, boolean speedBoost) {
        drive(chassisSpeeds.vxMetersPerSecond, 
            chassisSpeeds.vyMetersPerSecond, 
            chassisSpeeds.omegaRadiansPerSecond, 
            fieldRelative, false, speedBoost
        );
    }

    public abstract void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit, boolean speedBoost);

    public void drive(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, NeoDrivetrainConstants.MAX_SPEED_METERS_PER_SECOND);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_rearLeft.setDesiredState(desiredStates[2]);
        m_rearRight.setDesiredState(desiredStates[3]);       
    }

    public void resetEncoders() {
        m_frontLeft.resetEncoders();
		m_rearLeft.resetEncoders();
		m_frontRight.resetEncoders();
		m_rearRight.resetEncoders();
    }

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

    public SwerveModulePosition[] getSwerveModulePositions() {
        return new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        };
    }

    public SwerveModuleState[] getSwerveModuleStates() {
        return new SwerveModuleState[] {
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_rearLeft.getState(),
            m_rearRight.getState()
        };
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        ChassisSpeeds output = Constants.NeoDrivetrainConstants.DRIVE_KINEMATICS.toChassisSpeeds(getSwerveModuleStates());
        return output;
    }

    public void updatePIDControllers() {
        m_frontLeft.updatePIDControllers();
        m_frontRight.updatePIDControllers();
        m_rearLeft.updatePIDControllers();
        m_rearRight.updatePIDControllers();
    }
}
