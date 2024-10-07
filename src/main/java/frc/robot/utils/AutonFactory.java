package frc.robot.utils;

import java.util.List;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SwerveDrivetrainNew.BrandNewDrive;

public class AutonFactory extends VirtualSubsystem{

    private final BrandNewDrive m_drivetrain;
    private final LoggedDashboardChooser<String> m_autonChooser;
    private String m_currAutonChoice;
    private Command m_currentAutonCommand;
    
    public AutonFactory(BrandNewDrive drivetrain) {
        this.m_drivetrain = drivetrain;

        AutoBuilder.configureHolonomic(
            m_drivetrain::getPose, // Robot pose supplier
            m_drivetrain::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            m_drivetrain::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            m_drivetrain::setPathPlannerDesired, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(3.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(3.0, 0.0, 0.0), // Rotation PID constants
                    4.5, // Max module speed, in m/s
                    0.377, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            m_drivetrain // Reference to this subsystem to set requirements
        );

        m_autonChooser = new LoggedDashboardChooser<>("Auton Chooser");
        m_autonChooser.addOption("Do Nothing", "Do Nothing");
        List<String> paths = PathPlannerUtil.getExistingPaths();
        for (String path : paths) {
            m_autonChooser.addOption(path, path);
        }

        m_currAutonChoice = m_autonChooser.get();
        m_currentAutonCommand = buildAutonCommand(m_currAutonChoice);
    }

    private Command buildAutonCommand(String path) {
        if (path == null || path.equals("Do Nothing")) {
            return Commands.none();
        }
        Command autoCommand = AutoBuilder.buildAuto(path);
        return autoCommand;
    }

    @Override
    public void periodic() {
        String newAutonChoice = m_autonChooser.get();
        if (newAutonChoice == null && m_currAutonChoice == null) return;
        if (newAutonChoice == null || !newAutonChoice.equals(m_currAutonChoice)) {
            m_currAutonChoice = newAutonChoice;
            m_currentAutonCommand = buildAutonCommand(m_currAutonChoice);
        }
    }

    public Command getAutonCommand() {
        if (m_currentAutonCommand.isFinished()) {
            m_currentAutonCommand = buildAutonCommand(m_currAutonChoice);
        }

        return m_currentAutonCommand;
    }
}
