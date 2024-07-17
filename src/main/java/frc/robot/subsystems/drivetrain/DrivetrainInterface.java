package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public interface DrivetrainInterface {
    void configureDefaultCommand(CommandXboxController driverController);
    void resetHeading();
}