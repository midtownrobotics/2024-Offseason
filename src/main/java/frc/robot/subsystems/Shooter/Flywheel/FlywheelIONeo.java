package frc.robot.subsystems.Shooter.Flywheel;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.TempuratureConverter;
import org.littletonrobotics.junction.Logger;

public class FlywheelIONeo implements FlywheelIO {
  private SparkMax leftWheelNeo;
  private SparkMax rightWheelNeo;
  private SparkClosedLoopController leftWheelPID;
  private SparkClosedLoopController rightWheelPID;

  SparkMaxConfig leftWheelConfig = new SparkMaxConfig();
  SparkMaxConfig rightWheelConfig = new SparkMaxConfig();

  public FlywheelIONeo(int leftWheelNeoID, int rightWheelNeoID) {

    leftWheelNeo = new SparkMax(leftWheelNeoID, MotorType.kBrushless);
    leftWheelConfig.idleMode(IdleMode.kCoast);
    leftWheelConfig.smartCurrentLimit(MotorConstants.CURRENT_LIMIT_1650);
    leftWheelConfig.closedLoop.p(ShooterConstants.FLYWHEEL_SPEED_P.get());
    leftWheelConfig.closedLoop.i(ShooterConstants.FLYWHEEL_SPEED_I.get());
    leftWheelConfig.closedLoop.d(ShooterConstants.FLYWHEEL_SPEED_D.get());
    leftWheelConfig.closedLoop.velocityFF(ShooterConstants.FLYWHEEL_SPEED_FF.get());
    leftWheelConfig.closedLoop.outputRange(0, 1);
    leftWheelNeo.configure(leftWheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    rightWheelNeo = new SparkMax(rightWheelNeoID, MotorType.kBrushless);
    rightWheelConfig.idleMode(IdleMode.kCoast);
    rightWheelConfig.smartCurrentLimit(MotorConstants.CURRENT_LIMIT_1650);
    rightWheelConfig.closedLoop.p(ShooterConstants.FLYWHEEL_SPEED_P.get());
    rightWheelConfig.closedLoop.i(ShooterConstants.FLYWHEEL_SPEED_I.get());
    rightWheelConfig.closedLoop.d(ShooterConstants.FLYWHEEL_SPEED_D.get());
    rightWheelConfig.closedLoop.velocityFF(ShooterConstants.FLYWHEEL_SPEED_FF.get());
    rightWheelConfig.closedLoop.outputRange(0, 1);
    rightWheelNeo.configure(rightWheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    leftWheelPID = leftWheelNeo.getClosedLoopController();

    rightWheelPID = rightWheelNeo.getClosedLoopController();
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.leftOutputVoltage = leftWheelNeo.getBusVoltage() * leftWheelNeo.getAppliedOutput();
    inputs.rightOutputVoltage = rightWheelNeo.getBusVoltage() * rightWheelNeo.getAppliedOutput();
    inputs.leftIsOn = Math.abs(leftWheelNeo.getAppliedOutput()) > 0.01;
    inputs.rightIsOn = Math.abs(rightWheelNeo.getAppliedOutput()) > 0.01;
    inputs.leftVelocityRPM = leftWheelNeo.getEncoder().getVelocity();
    inputs.rightVelocityRPM = rightWheelNeo.getEncoder().getVelocity();
    inputs.leftTempFahrenheit =
        TempuratureConverter.celsiusToFahrenheit(leftWheelNeo.getMotorTemperature());
    inputs.rightTempFahrenheit =
        TempuratureConverter.celsiusToFahrenheit(rightWheelNeo.getMotorTemperature());
    inputs.leftCurrentAmps = leftWheelNeo.getOutputCurrent();
    inputs.rightCurrentAmps = rightWheelNeo.getOutputCurrent();
  }

  @Override
  public void setSpeed(double leftSpeed, double rightSpeed) {

    Logger.recordOutput("Shooter/DesiredLeftSpeed", leftSpeed);
    Logger.recordOutput("Shooter/DesiredRightSpeed", rightSpeed);

    if (leftSpeed == 0) {
      leftWheelPID.setReference(0, ControlType.kDutyCycle);
    } else {
      leftWheelPID.setReference(leftSpeed, ControlType.kVelocity);
    }

    if (rightSpeed == 0) {
      rightWheelPID.setReference(0, ControlType.kDutyCycle);
    } else {
      rightWheelPID.setReference(rightSpeed, ControlType.kVelocity);
    }
  }

  public void updatePIDControllers() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          leftWheelConfig.closedLoop.p(ShooterConstants.FLYWHEEL_SPEED_P.get());
          leftWheelConfig.closedLoop.i(ShooterConstants.FLYWHEEL_SPEED_I.get());
          leftWheelConfig.closedLoop.d(ShooterConstants.FLYWHEEL_SPEED_D.get());
          leftWheelConfig.closedLoop.velocityFF(ShooterConstants.FLYWHEEL_SPEED_FF.get());
          leftWheelNeo.configure(leftWheelConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        },
        ShooterConstants.FLYWHEEL_SPEED_P,
        ShooterConstants.FLYWHEEL_SPEED_I,
        ShooterConstants.FLYWHEEL_SPEED_D,
        ShooterConstants.FLYWHEEL_SPEED_FF);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          rightWheelConfig.closedLoop.p(ShooterConstants.FLYWHEEL_SPEED_P.get());
          rightWheelConfig.closedLoop.i(ShooterConstants.FLYWHEEL_SPEED_I.get());
          rightWheelConfig.closedLoop.d(ShooterConstants.FLYWHEEL_SPEED_D.get());
          rightWheelConfig.closedLoop.velocityFF(ShooterConstants.FLYWHEEL_SPEED_FF.get());
          leftWheelNeo.configure(rightWheelConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        },
        ShooterConstants.FLYWHEEL_SPEED_P,
        ShooterConstants.FLYWHEEL_SPEED_I,
        ShooterConstants.FLYWHEEL_SPEED_D,
        ShooterConstants.FLYWHEEL_SPEED_FF);
  }

    /** Get the speed of the fastest spinning flywheel.
     * @return The speed of the fastest spinning flywheel  
    **/

    public double getSpeed() {
        return Math.max(leftWheelNeo.getEncoder().getVelocity(), rightWheelNeo.getEncoder().getVelocity());
    }
}
