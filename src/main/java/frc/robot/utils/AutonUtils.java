package frc.robot.utils;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrivetrainNew.SwerveDrivetrainNew;

public class AutonUtils {
    private SwerveDrivetrainNew drivetrain;
    private SendableChooser<Command> autonChooser;

    public AutonUtils(SwerveDrivetrainNew drivetrain) {
        this.drivetrain = drivetrain;

        AutoBuilder.configureHolonomic(
            drivetrain::getPose, // Robot pose supplier
            drivetrain::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            drivetrain::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(0.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(0.0, 0.0, 0.0), // Rotation PID constants
                    4.5, // Max module speed, in m/s
                    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            drivetrain // Reference to this subsystem to set requirements
        );
        
        autonChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Auton Chooser", autonChooser);

    }

    public Command getPathPlannerAuton() {
        return autonChooser.getSelected();
    }

    private void drive(ChassisSpeeds chassisSpeeds) {
        Logger.recordOutput("desiredSpeeds", chassisSpeeds);
        drivetrain.drive(chassisSpeeds, false);
    }
}
