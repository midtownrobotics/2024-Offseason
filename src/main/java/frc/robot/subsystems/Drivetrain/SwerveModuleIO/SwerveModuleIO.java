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
  }

  public void updateInputs(SwerveModuleIOInputs inputs);

  /**
   * what we want the modules to do
   * @return what we want the modules to do
   */
  public SwerveModuleState getDesiredState();

  /**
   * tell the modules what to do
   * @param desiredState what we want the modules to do
   */
  public void setDesiredState(SwerveModuleState desiredState);

  /**
   * get the state of the module
   * @return the state of the module
   */
  public SwerveModuleState getState();

  /**
   * get position of the module
   * @return the position of the module
   */
  public SwerveModulePosition getPosition();
}
