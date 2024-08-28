package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public interface DrivetrainInterface extends Subsystem {
    void configureDefaultCommand(CommandXboxController driverController);
    void resetHeading();
    void setX();
    void setBoost(boolean boost);
}