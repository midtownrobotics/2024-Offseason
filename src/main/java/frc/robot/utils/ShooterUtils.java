package frc.robot.utils;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterUtils {

  public InterpolatingDoubleTreeMap lookupTableShooterAngle;
  public static ShooterUtils instance = new ShooterUtils();

  public ShooterUtils() {
    lookupTableShooterAngle = new InterpolatingDoubleTreeMap();
    lookupTableShooterAngle.put(159.4, 0.88);
    lookupTableShooterAngle.put(182.0, 0.92);
    lookupTableShooterAngle.put(199.0, 0.94);
    lookupTableShooterAngle.put(221.0, 0.97);
    lookupTableShooterAngle.put(231.0, 0.988);
  }

  public double getAngleFromDistance(double distance) {
    return lookupTableShooterAngle.get(distance);
  }
}
