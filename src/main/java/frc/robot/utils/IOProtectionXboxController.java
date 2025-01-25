package frc.robot.utils;

import edu.wpi.first.wpilibj.XboxController;

public class IOProtectionXboxController extends XboxController {
  public double currentRumbleValue;

  public IOProtectionXboxController(int port) {
    super(port);
  }

  public void setRumble(RumbleType rumbleType, double value) {
    if (currentRumbleValue != value) {
      super.setRumble(rumbleType, value);
      currentRumbleValue = value;
    }
  }
}
