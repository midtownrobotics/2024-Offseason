// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain.Drivetrain;
import frc.robot.subsystems.Drivetrain.Drivetrain.DriveState;
import frc.robot.utils.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class WheelRadiusCharacterization extends Command {
  private final Drivetrain drive;
  private static final LoggedTunableNumber characterizationSpeed =
      new LoggedTunableNumber("WheelRadiusCharacterization/SpeedRadsPerSec", 0.1);
  private static final double driveRadius =
      Math.hypot(
          Constants.NeoDrivetrainConstants.WHEEL_BASE_METERS / 2,
          Constants.NeoDrivetrainConstants.WHEEL_BASE_METERS / 2);
  private final DoubleSupplier gyroYawRadsSupplier;

  // @RequiredArgsConstructor
  public enum Direction {
    CLOCKWISE(-1),
    COUNTER_CLOCKWISE(1);

    private final int value;

    Direction(int value) {
      this.value = value;
    }
  }

  private final Direction omegaDirection;
  private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(1.0);

  private double lastGyroYawRads = 0.0;
  private double accumGyroYawRads = 0.0;

  private double[] startWheelPositions;

  private double currentEffectiveWheelRadius = 0.0;

  public WheelRadiusCharacterization(Drivetrain drive, Direction omegaDirection) {
    this.drive = drive;
    this.omegaDirection = omegaDirection;
    this.gyroYawRadsSupplier = () -> Units.degreesToRadians(drive.getAngle());
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    // Reset
    lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
    accumGyroYawRads = 0;
    currentEffectiveWheelRadius = 0;

    startWheelPositions = drive.getWheelRadiusCharacterizationPosition();

    omegaLimiter.reset(0);
  }

  @Override
  public void execute() {
    // Run drive at velocity
    drive.runWheelRadiusCharacterization(
        omegaLimiter.calculate(omegaDirection.value * characterizationSpeed.get()));

    // Get yaw and wheel positions
    // accumGyroYawRads = gyroYawRadsSupplier.getAsDouble() - lastGyroYawRads;
    // accumGyroYawRads += MathUtil.angleModulus(gyroYawRadsSupplier.getAsDouble() -
    // lastGyroYawRads);
    // lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();

    accumGyroYawRads = gyroYawRadsSupplier.getAsDouble() - lastGyroYawRads;
    double averageWheelPosition = 0.0;
    double[] wheelPositiions = drive.getWheelRadiusCharacterizationPosition();
    double[] relWheelPositions = new double[4];
    for (int i = 0; i < 4; i++) {
      averageWheelPosition += Math.abs(wheelPositiions[i] - startWheelPositions[i]);
      relWheelPositions[i] = wheelPositiions[i] - startWheelPositions[i];
    }

    Logger.recordOutput("Drive/RadiusCharacterization/Positions", relWheelPositions);
    averageWheelPosition /= 4.0;

    currentEffectiveWheelRadius = (accumGyroYawRads * driveRadius) / averageWheelPosition;
    Logger.recordOutput("Drive/RadiusCharacterization/DrivePosition", averageWheelPosition);
    Logger.recordOutput("Drive/RadiusCharacterization/AccumGyroYawRads", accumGyroYawRads);
    Logger.recordOutput(
        "Drive/RadiusCharacterization/CurrentWheelRadiusInches",
        Units.metersToInches(currentEffectiveWheelRadius));
    Logger.recordOutput("Drive/RadiusCharacterization/Gyro", gyroYawRadsSupplier.getAsDouble());

    if (accumGyroYawRads > 4 * Math.PI) {
      cancel();
    }
  }

  @Override
  public void end(boolean interrupted) {
    drive.setState(DriveState.MANUAL);
    if (accumGyroYawRads <= Math.PI * 2.0) {
      System.out.println("Not enough data for characterization");
    } else {
      System.out.println(
          "Effective Wheel Radius: "
              + Units.metersToInches(currentEffectiveWheelRadius)
              + " inches");
    }
  }
}
