package frc.robot.subsystems.Drivetrain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain.SwerveDrivetrainIO.SwerveDrivetrainIO;
import frc.robot.subsystems.Drivetrain.SwerveDrivetrainIO.SwerveIOInputsAutoLogged;
import frc.robot.subsystems.Limelight.Limelight;

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

    private ChassisSpeeds driverChassisSpeeds; // Robot Relative
    private ChassisSpeeds pathplannerChassisSpeeds; // Robot Relative

    private PIDController autoAimPID = new PIDController(0.02, 0, 0.001);

    private boolean speedBoost;

    public BrandNewDrive(SwerveDrivetrainIO swerveDrivetrainIO, Limelight limelight) {
        m_swerveDrivetrainIO = swerveDrivetrainIO;
        m_limelight = limelight;

        autoAimPID.setSetpoint(0);
    }

    @Override
    public void periodic() {
        m_swerveDrivetrainIO.updatePIDControllers();

        m_swerveDrivetrainIO.updateOdometry();
        if (!state.equals(DriveState.FOLLOW_PATH)) {
            m_swerveDrivetrainIO.updateOdometryWithVision(m_limelight);
        }

        switch (state) {
            case SPEAKER_AUTO_ALIGN:
                if (m_limelight.isValidTarget(7)) {
                    m_swerveDrivetrainIO.drive(
                        driverChassisSpeeds.vxMetersPerSecond,
                        driverChassisSpeeds.vyMetersPerSecond,
                        -autoAimPID.calculate(m_limelight.getTx()), 
                        false, false, speedBoost);
                    break;
                }

                // Intentional Fall-through - if Limelight does not detect target, we do manual driving
            case TUNING:
            case MANUAL:
                m_swerveDrivetrainIO.drive(driverChassisSpeeds, true, speedBoost);
                break;
            case FOLLOW_PATH:
                m_swerveDrivetrainIO.drive(pathplannerChassisSpeeds, false, true);
                break;
            case X:
                // m_swerveDrivetrainIO.drive(4, 
                //     0, 
                //     0, 
                //     true, false, speedBoost
                // );
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
        // swerveIOInputs.pose = getPose();
        Logger.processInputs("Drive", swerveIOInputs);
    }

    public void setState(DriveState state) {
        this.state = state;
    }

    public void setBoost(boolean boost) {
        speedBoost = boost;
    }

    public void setDriverDesired(ChassisSpeeds speeds) {
        driverChassisSpeeds = speeds;
    }

    public void setPathPlannerDesired(ChassisSpeeds speeds) {
        pathplannerChassisSpeeds = speeds;
        Logger.recordOutput("Drive/PathPlannerSpeed", pathplannerChassisSpeeds);
    }

    public double getAngle() {
        return m_swerveDrivetrainIO.getPigeonYaw();
    }

    public void resetHeading() {
        m_swerveDrivetrainIO.resetHeading();
    }

    public Pose2d getPose() {
        return m_swerveDrivetrainIO.getPose();
    }

    public void resetOdometry(Pose2d pose) {
        m_swerveDrivetrainIO.resetOdometry(pose);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        ChassisSpeeds output = Constants.NeoDrivetrainConstants.DRIVE_KINEMATICS.toChassisSpeeds(
            m_swerveDrivetrainIO.getSwerveModuleStates()
        );
        return output;
    }
    
}
