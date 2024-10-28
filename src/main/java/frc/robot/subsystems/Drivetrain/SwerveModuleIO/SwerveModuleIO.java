package frc.robot.subsystems.Drivetrain.SwerveModuleIO;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {

  @AutoLog
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

    public SwerveModuleState currentState;
    public SwerveModuleState desiredState;

    public double offset;
    public double turningEncoderPosition;
    public double turningAbsolutePosition;
  }

  public void updateInputs(SwerveModuleIOInputs inputs);

  public SwerveModuleState getDesiredState();

  public void setDesiredState(SwerveModuleState desiredState);

  public SwerveModuleState getState();

  public SwerveModulePosition getPosition();
}
