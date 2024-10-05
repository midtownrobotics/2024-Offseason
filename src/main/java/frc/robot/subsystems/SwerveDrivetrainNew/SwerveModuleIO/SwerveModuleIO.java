package frc.robot.subsystems.SwerveDrivetrainNew.SwerveModuleIO;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModuleIO {
    
    public class SwerveModuleIOInputs {
        public double turningCurrentAmps;
        public double turningTempFahrenheit;
        public double turningVelocityRPM;
        public boolean turningIsOn;
        public double turningVoltage;
        public double drivingCurrentAmps;
        public double drivingTempFahrenheit;
        public double drivingVelocityRPM;
        public boolean drivingIsOn;
        public double drivingVoltage;

        public SwerveModuleState currenState;
        public SwerveModuleState desiredState;
    }

    public void updateInputs(SwerveModuleIOInputs inputs);

    public void resetEncoders();

    public void calibrateVirtualPosition(double angle);

    public double getOffset();

    public RelativeEncoder getDrivingEncoder();

    public RelativeEncoder getTurningEncoder();

    public CANCoder getTurningAbsoluteEncoder();

    public SwerveModuleState getDesiredState();

    public void setDesiredState(SwerveModuleState desiredState);

    public SwerveModuleState getState();

    public SwerveModulePosition getPosition();

    // getDrivingTemp
    // getTurningTemp

    public void updatePIDControllers();





}
