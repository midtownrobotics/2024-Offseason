package frc.robot.subsystems.SwerveDrivetrainNew;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Limelight.Limelight;
import frc.robot.subsystems.drivetrain.NeoSwerveDrive.NeoSwerveDrivetrain;
import frc.robot.utils.LoggedTunableNumber;

public class SwerveDrivetrainNew extends SubsystemBase {
    private NeoSwerveDrivetrain drivetrain = new NeoSwerveDrivetrain();

    public enum SwerveDriveState {
        MANUAL,
        AUTO,
        SPEAKER_AUTO_ALIGN,
        X
    }   

    private Limelight limelight;

    private SwerveDriveState state = SwerveDriveState.MANUAL;

    private ChassisSpeeds driverChassisSpeeds;
    private ChassisSpeeds autoChassisSpeeds;

    private PIDController autoAimPID = new PIDController(0.02, 0, 0.001);

    public SwerveDrivetrainNew(Limelight limelight) {
        this.limelight = limelight;
        autoAimPID.setSetpoint(0);
    }

    @Override
    public void periodic() {
        switch (state) {
            case MANUAL:
                if (driverChassisSpeeds == null) break;
                drivetrain.drive(driverChassisSpeeds);
                break;
            case AUTO:
                
                break;
            case SPEAKER_AUTO_ALIGN:
                if (driverChassisSpeeds == null || !limelight.isValidTarget(new String[]{"7"})) break;
                drivetrain.drive(
                    driverChassisSpeeds.vxMetersPerSecond,
                    driverChassisSpeeds.vyMetersPerSecond,
                    -autoAimPID.calculate(limelight.getTx()), 
                   true
                );
                break;
            case X:
                drivetrain.setX();
                break;
        }

        LoggedTunableNumber.ifChanged(hashCode(), () -> {
            autoAimPID.setP(Constants.AUTOAIM_P.get());
        });


        Logger.recordOutput("AutoAim/PIDOutput", autoAimPID.calculate(limelight.getTx()));
        Logger.recordOutput("AutoAim/PID_P", autoAimPID.getP());
        Logger.recordOutput("AutoAim/PID_D", autoAimPID.getD());
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

    public void setDriverDesiredSpeeds(ChassisSpeeds driverChassisSpeeds) {
        this.driverChassisSpeeds = driverChassisSpeeds;
    }

    public void setAutoDesiredSpeeds(ChassisSpeeds autoChassisSpeeds) {
        this.autoChassisSpeeds = autoChassisSpeeds;
    }

    public void setState(SwerveDriveState state) {
        this.state = state;
    }

    public void resetHeading() {
        drivetrain.resetHeading();
    }

    public void setBoost(boolean boost) {
        drivetrain.setBoost(boost);
    }
}
