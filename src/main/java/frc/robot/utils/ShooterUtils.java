package frc.robot.utils;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.Constants;

public class ShooterUtils {

  public InterpolatingDoubleTreeMap lookupTableShooterAngle;
  public static ShooterUtils instance = new ShooterUtils();

  public ShooterUtils() {
    lookupTableShooterAngle = new InterpolatingDoubleTreeMap();
    lookupTableShooterAngle.put(159.4, 0.852);
    lookupTableShooterAngle.put(181.0, 0.87);
    lookupTableShooterAngle.put(202.0, 0.9);
    lookupTableShooterAngle.put(220.0, 0.92);
    lookupTableShooterAngle.put(240.0, 0.945); // 3800 RPM
    lookupTableShooterAngle.put(260.0, 0.98);
    lookupTableShooterAngle.put(280.0, 1.0);
    lookupTableShooterAngle.put(300.0, 1.03);
    lookupTableShooterAngle.put(320.0, 1.06);
    lookupTableShooterAngle.put(340.0, 1.08);
    lookupTableShooterAngle.put(360.0, 1.09);
  }

  public double getAngleFromDistance(double distance) {
    return lookupTableShooterAngle.get(distance);
  }

  public double getSpeedFromDistance(double distance) {
    if (distance >= 230) return 3800;
    return Constants.ShooterConstants.SPEAKER_SPEED.get();
  }
}
