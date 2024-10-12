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

    public void resetHeading();

    default void drive(ChassisSpeeds chassisSpeeds, boolean fieldRelative, boolean speedBoost) {
        if (chassisSpeeds == null) {
            chassisSpeeds = new ChassisSpeeds();
        }

        /** This is a slight weakness, since it will only be applied to this method */
        chassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.05);

        drive(chassisSpeeds.vxMetersPerSecond, 
            chassisSpeeds.vyMetersPerSecond, 
            chassisSpeeds.omegaRadiansPerSecond, 
            fieldRelative, false, speedBoost
        );
    }

    void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit, boolean speedBoost);

    default void drive(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, NeoDrivetrainConstants.MAX_SPEED_METERS_PER_SECOND);
        getFrontLeftModule().setDesiredState(desiredStates[0]);
        getFrontRightModule().setDesiredState(desiredStates[1]);
        getRearLeftModule().setDesiredState(desiredStates[2]);
        getRearRightModule().setDesiredState(desiredStates[3]);       
    }

    public abstract double getPigeonYaw();

    // ODOMETRY DONE BY Drive class
    
    SwerveModuleIO getFrontLeftModule();

	SwerveModuleIO getFrontRightModule();

	SwerveModuleIO getRearLeftModule();

	SwerveModuleIO getRearRightModule();

    default SwerveModulePosition[] getSwerveModulePositions() {
        return new SwerveModulePosition[] {
            getFrontLeftModule().getPosition(),
            getFrontRightModule().getPosition(),
            getRearLeftModule().getPosition(),
            getRearRightModule().getPosition()
        };
    }

    default SwerveModuleState[] getSwerveModuleStates() {
        return new SwerveModuleState[] {
            getFrontLeftModule().getState(),
            getFrontRightModule().getState(),
            getRearLeftModule().getState(),
            getRearRightModule().getState()
        };
    }

    default SwerveModuleState[] getSwerveModuleDesiredStates() {
        return new SwerveModuleState[] {
            getFrontLeftModule().getDesiredState(),
            getFrontRightModule().getDesiredState(),
            getRearLeftModule().getDesiredState(),
            getRearRightModule().getDesiredState()
        };
    }

    default ChassisSpeeds getRobotRelativeSpeeds() {
        ChassisSpeeds output = Constants.NeoDrivetrainConstants.DRIVE_KINEMATICS.toChassisSpeeds(getSwerveModuleStates());
        return output;
    }

    void updatePIDControllers();

    Pose2d getPose();

    void resetOdometry(Pose2d pose);

    void updateOdometry();

    void updateOdometryWithVision(Limelight limelight);
}
