package frc.robot.subsystems.Shooter.Flywheel;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.utils.TempuratureConverter;
import org.littletonrobotics.junction.Logger;

public class FlywheelIONeo implements FlywheelIO {
  private SparkMax leftWheelNeo;
  private SparkMax rightWheelNeo;
  private SparkClosedLoopController leftWheelPID;
  private SparkClosedLoopController rightWheelPID;

  public FlywheelIONeo(int leftWheelNeoID, int rightWheelNeoID) {

    SparkBaseConfig leftWheelConfig =
        new SparkMaxConfig()
            .idleMode(IdleMode.kCoast)
            .inverted(true)
            .smartCurrentLimit(MotorConstants.CURRENT_LIMIT_1650);

    leftWheelConfig
        .closedLoop
        .p(ShooterConstants.FLYWHEEL_SPEED_P.get())
        .i(ShooterConstants.FLYWHEEL_SPEED_I.get())
        .d(ShooterConstants.FLYWHEEL_SPEED_D.get())
        .velocityFF(ShooterConstants.FLYWHEEL_SPEED_FF.get())
        .outputRange(0, 1);

    leftWheelNeo = new SparkMax(leftWheelNeoID, MotorType.kBrushless);
    leftWheelNeo.configure(
        leftWheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkBaseConfig rightWheelConfig =
        new SparkMaxConfig()
            .idleMode(IdleMode.kCoast)
            .inverted(false)
            .smartCurrentLimit(MotorConstants.CURRENT_LIMIT_1650);

    rightWheelConfig
        .closedLoop
        .p(ShooterConstants.FLYWHEEL_SPEED_P.get())
        .i(ShooterConstants.FLYWHEEL_SPEED_I.get())
        .d(ShooterConstants.FLYWHEEL_SPEED_D.get())
        .velocityFF(ShooterConstants.FLYWHEEL_SPEED_FF.get())
        .outputRange(0, 1);

    rightWheelNeo = new SparkMax(rightWheelNeoID, MotorType.kBrushless);
    rightWheelNeo.configure(
        rightWheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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
    // LoggedTunableNumber.ifChanged(
    //     hashCode(),
    //     () -> {
    //       leftWheelPID.setP(ShooterConstants.FLYWHEEL_SPEED_P.get());
    //       leftWheelPID.setI(ShooterConstants.FLYWHEEL_SPEED_I.get());
    //       leftWheelPID.setD(ShooterConstants.FLYWHEEL_SPEED_D.get());
    //       leftWheelPID.setFF(ShooterConstants.FLYWHEEL_SPEED_FF.get());
    //     },
    //     ShooterConstants.FLYWHEEL_SPEED_P,
    //     ShooterConstants.FLYWHEEL_SPEED_I,
    //     ShooterConstants.FLYWHEEL_SPEED_D,
    //     ShooterConstants.FLYWHEEL_SPEED_FF);

    // LoggedTunableNumber.ifChanged(
    //     hashCode(),
    //     () -> {
    //       rightWheelPID.setP(ShooterConstants.FLYWHEEL_SPEED_P.get());
    //       rightWheelPID.setI(ShooterConstants.FLYWHEEL_SPEED_I.get());
    //       rightWheelPID.setD(ShooterConstants.FLYWHEEL_SPEED_D.get());
    //       rightWheelPID.setFF(ShooterConstants.FLYWHEEL_SPEED_FF.get());
    //     },
    //     ShooterConstants.FLYWHEEL_SPEED_P,
    //     ShooterConstants.FLYWHEEL_SPEED_I,
    //     ShooterConstants.FLYWHEEL_SPEED_D,
    //     ShooterConstants.FLYWHEEL_SPEED_FF);
  }

  /**
   * Get the speed of the fastest spinning flywheel.
   *
   * @return The speed of the fastest spinning flywheel
   */
  public double getSpeed() {
    return Math.max(
        leftWheelNeo.getEncoder().getVelocity(), rightWheelNeo.getEncoder().getVelocity());
  }
}
