package frc.robot.subsystems.SwerveDrivetrainNew;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drivetrain.NeoSwerveDrive.NeoSwerveDrivetrain;

public class SwerveDrivetrainNew extends SubsystemBase {
    private NeoSwerveDrivetrain drivetrain = new NeoSwerveDrivetrain();

    public enum SwerveDriveState {
        MANUAL,
        AUTO,
        SPEAKER_AUTO_ALIGN,
        X
    }

    private SwerveDriveState state = SwerveDriveState.MANUAL;

    private ChassisSpeeds driverChassisSpeeds;
    private ChassisSpeeds autoChassisSpeeds;

    public SwerveDrivetrainNew() {

    }

    public void setDriverDesiredSpeeds(ChassisSpeeds driverChassisSpeeds) {
        this.driverChassisSpeeds = driverChassisSpeeds;
    }

    public void setAutoDesiredSpeeds(ChassisSpeeds autoChassisSpeeds) {
        this.autoChassisSpeeds = autoChassisSpeeds;
    }

    public void setState(SwerveDriveState state) {
        this.state = state;
    }

    public void configureDefaultCommand(CommandXboxController driver) {
        drivetrain.setDefaultCommand(new RunCommand (() -> {
            driverChassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                RobotContainer.deadzone(driver.getLeftY(), driver.getLeftX(), driver.getRightX(), Constants.JOYSTICK_THRESHOLD)*Constants.CONTROL_LIMITER,
                RobotContainer.deadzone(driver.getLeftX(), driver.getLeftY(), driver.getRightX(), Constants.JOYSTICK_THRESHOLD)*Constants.CONTROL_LIMITER,
                RobotContainer.deadzone(driver.getRightX(), driver.getLeftY(), driver.getLeftX(), Constants.JOYSTICK_THRESHOLD)*Constants.CONTROL_LIMITER,
                drivetrain.getRotation2d()
            );
        }, drivetrain));
    }

    @Override
    public void periodic() {
        switch (state) {
            case MANUAL:
                if (driverChassisSpeeds == null) break;
                drivetrain.drive(driverChassisSpeeds.vxMetersPerSecond, driverChassisSpeeds.vyMetersPerSecond, driverChassisSpeeds.omegaRadiansPerSecond, true);
                break;
            case AUTO:
                
                break;
            case SPEAKER_AUTO_ALIGN:
                if (driverChassisSpeeds == null) break;
                drivetrain.drive(driverChassisSpeeds.vxMetersPerSecond, driverChassisSpeeds.vyMetersPerSecond, 0, true);
                break;
            case X:
                drivetrain.setX();
                break;
        }
    }

    public void resetHeading() {
        drivetrain.resetHeading();
    }

    public void setBoost(boolean boost) {
        drivetrain.setBoost(boost);
    }
}
