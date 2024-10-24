package frc.robot.utils;

import edu.wpi.first.wpilibj.XboxController;

public class IOProtectionXboxController extends XboxController{
    public boolean rumbleLastCycle;
    public IOProtectionXboxController(int port) {
        super(port);
    }

    public void setRumble(RumbleType rumbleType, double value) {
        if (!rumbleLastCycle) {
            super.setRumble(rumbleType, value);
            rumbleLastCycle = true;
        } else {
            rumbleLastCycle = false;
        }
    }
}