package frc.robot.subsystems.Shooter.Flywheel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class FlywheelIOSim implements FlywheelIO {
  private final DCMotorSim leftWheelSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(0.1, 0.1),
          DCMotor.getNEO(1),
          Constants.ShooterConstants.Simulation.FLYWHEEL_GEARING.get(),
          0.01);

  private final DCMotorSim rightWheelSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(0.1, 0.1),
          DCMotor.getNEO(1),
          Constants.ShooterConstants.Simulation.FLYWHEEL_GEARING.get(),
          0.01);

  @Override
  public void setSpeed(double leftSpeed, double rightSpeed) {
    leftWheelSim.setInputVoltage(leftSpeed * Constants.THEORETICAL_RESTING_VOLTAGE);
    rightWheelSim.setInputVoltage(rightSpeed * Constants.THEORETICAL_RESTING_VOLTAGE);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    leftWheelSim.update(0.02);
    rightWheelSim.update(0.02);

    inputs.rightOutputVoltage = MathUtil.clamp(leftWheelSim.getOutput(0), -12.0, 12.0);
    inputs.leftOutputVoltage = MathUtil.clamp(rightWheelSim.getOutput(0), -12.0, 12.0);

    inputs.rightIsOn = leftWheelSim.getAngularVelocityRPM() > 0.01;
    inputs.leftIsOn = rightWheelSim.getAngularVelocityRPM() > 0.01;

    inputs.rightVelocityRPM = leftWheelSim.getAngularVelocityRPM();
    inputs.leftVelocityRPM = rightWheelSim.getAngularVelocityRPM();

    inputs.leftTempFahrenheit = 0.0;
    inputs.rightTempFahrenheit = 0.0;

    inputs.rightCurrentAmps = leftWheelSim.getCurrentDrawAmps();
    inputs.leftCurrentAmps = rightWheelSim.getCurrentDrawAmps();
  }

  public void updatePIDControllers() {}

  public double getSpeed() {
    return 0.0;
  }
}
