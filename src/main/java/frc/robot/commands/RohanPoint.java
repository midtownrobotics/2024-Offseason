package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain.Drivetrain;
import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.LoggedTunablePIDController;

public class RohanPoint extends Command{

    public static final double kMaxLinearSpeed = 2.0; // meters per second
    public static final double kMaxLinearAcceleration = 3; // meters per second squared
    public static final double kTrackWidthX = Units.inchesToMeters(15.25);
    public static final double kTrackWidthY = Units.inchesToMeters(16.25);
    public static final double kDriveBaseRadius =
        Math.hypot(kTrackWidthX / 2.0, kTrackWidthY / 2.0);
    public static final double kMaxAngularSpeed = 0.9;
    public static final double kMaxAngularAcceleration = 0.9;

    private Drivetrain m_drive;
    private Pose2d m_targetPose;
    private LoggedTunablePIDController m_xController = new LoggedTunablePIDController("/RohanPoint/X", 2, 0, 0.15);
    private LoggedTunablePIDController m_yController = new LoggedTunablePIDController("/RohanPoint/Y", 2, 0, 0.15);
    private LoggedTunablePIDController m_thetaController = new LoggedTunablePIDController("/RohanPoint/Theta", 1, 0, .1);

    public RohanPoint(Drivetrain drive, Pose2d targetPose) {
        m_drive = drive;
        m_targetPose = targetPose;
        addRequirements(m_drive);

        m_thetaController.getController().enableContinuousInput(-Math.PI, Math.PI);

        m_xController.getController().setTolerance(Units.inchesToMeters(0.25));
        m_yController.getController().setTolerance(Units.inchesToMeters(0.25));
        m_thetaController.getController().setTolerance(Units.degreesToRadians(0.5));
    }

    @Override
    public void initialize() {

        m_xController.getController().reset();
        m_yController.getController().reset();
        m_thetaController.getController().reset();

    }

    @Override
    public void execute() {


        m_xController.updateValues();
        m_yController.updateValues();
        m_thetaController.updateValues();

        double thetaVelocity = 0;
        double xVelocity = 0;
        double yVelocity = 0;

        Pose2d currentPose = m_drive.getPose();

        if (!m_thetaController.getController().atSetpoint()) {
            thetaVelocity = m_thetaController.getController().calculate(currentPose.getRotation().getRadians(), m_targetPose.getRotation().getRadians());
        }
        if (!m_xController.getController().atSetpoint()) {
            xVelocity = m_xController.getController().calculate(currentPose.getX(), m_targetPose.getX());
        }
        if (!m_yController.getController().atSetpoint()) {
            yVelocity = m_yController.getController().calculate(currentPose.getY(), m_targetPose.getY());
        }

        ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(
            xVelocity, yVelocity, thetaVelocity
        ), m_drive.getPose().getRotation());

        m_drive.setDriveToPointDesired(robotRelativeSpeeds);

        Logger.recordOutput("RohanPoint/TargetPose", m_targetPose);
        
        Logger.recordOutput("RohanPoint/HeadingError", m_thetaController.getController().getPositionError());
        Logger.recordOutput("RohanPoint/HeadingVelocity", thetaVelocity);

        Logger.recordOutput("RohanPoint/XError", m_xController.getController().getPositionError());
        Logger.recordOutput("RohanPoint/XVelocity", xVelocity);

        Logger.recordOutput("RohanPoint/YError", m_yController.getController().getPositionError());
        Logger.recordOutput("RohanPoint/YVelocity", yVelocity);



    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return super.isFinished();
    }
    
}
