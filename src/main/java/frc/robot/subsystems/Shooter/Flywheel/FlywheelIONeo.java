package frc.robot.subsystems.Shooter.Flywheel;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.TempuratureConverter;
import org.littletonrobotics.junction.Logger;

public class FlywheelIONeo implements FlywheelIO {
  private CANSparkMax leftWheelNeo;
  private CANSparkMax rightWheelNeo;
  private SparkPIDController leftWheelPID;
  private SparkPIDController rightWheelPID;

  public FlywheelIONeo(int leftWheelNeoID, int rightWheelNeoID) {

    leftWheelNeo = new CANSparkMax(leftWheelNeoID, MotorType.kBrushless);
    leftWheelNeo.setIdleMode(IdleMode.kCoast);
    leftWheelNeo.setSmartCurrentLimit(MotorConstants.CURRENT_LIMIT_1650);
    leftWheelNeo.burnFlash();

    rightWheelNeo = new CANSparkMax(rightWheelNeoID, MotorType.kBrushless);
    rightWheelNeo.setIdleMode(IdleMode.kCoast);
    rightWheelNeo.setSmartCurrentLimit(MotorConstants.CURRENT_LIMIT_1650);
    rightWheelNeo.burnFlash();

    leftWheelPID = leftWheelNeo.getPIDController();
    leftWheelPID.setP(ShooterConstants.FLYWHEEL_SPEED_P.get());
    leftWheelPID.setI(ShooterConstants.FLYWHEEL_SPEED_I.get());
    leftWheelPID.setD(ShooterConstants.FLYWHEEL_SPEED_D.get());
    leftWheelPID.setFF(ShooterConstants.FLYWHEEL_SPEED_FF.get());
    leftWheelPID.setOutputRange(0, 1);

    rightWheelPID = rightWheelNeo.getPIDController();
    rightWheelPID.setP(ShooterConstants.FLYWHEEL_SPEED_P.get());
    rightWheelPID.setI(ShooterConstants.FLYWHEEL_SPEED_I.get());
    rightWheelPID.setD(ShooterConstants.FLYWHEEL_SPEED_D.get());
    rightWheelPID.setFF(ShooterConstants.FLYWHEEL_SPEED_FF.get());
    rightWheelPID.setOutputRange(0, 1);
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
          leftWheelPID.setP(ShooterConstants.FLYWHEEL_SPEED_P.get());
          leftWheelPID.setI(ShooterConstants.FLYWHEEL_SPEED_I.get());
          leftWheelPID.setD(ShooterConstants.FLYWHEEL_SPEED_D.get());
          leftWheelPID.setFF(ShooterConstants.FLYWHEEL_SPEED_FF.get());
        },
        ShooterConstants.FLYWHEEL_SPEED_P,
        ShooterConstants.FLYWHEEL_SPEED_I,
        ShooterConstants.FLYWHEEL_SPEED_D,
        ShooterConstants.FLYWHEEL_SPEED_FF);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          rightWheelPID.setP(ShooterConstants.FLYWHEEL_SPEED_P.get());
          rightWheelPID.setI(ShooterConstants.FLYWHEEL_SPEED_I.get());
          rightWheelPID.setD(ShooterConstants.FLYWHEEL_SPEED_D.get());
          rightWheelPID.setFF(ShooterConstants.FLYWHEEL_SPEED_FF.get());
        },
        ShooterConstants.FLYWHEEL_SPEED_P,
        ShooterConstants.FLYWHEEL_SPEED_I,
        ShooterConstants.FLYWHEEL_SPEED_D,
        ShooterConstants.FLYWHEEL_SPEED_FF);
  }
}
