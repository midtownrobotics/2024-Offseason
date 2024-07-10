package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public final class Constants {
    public static enum Mode {
        REAL,
        SIM,
        REPLAY
    }

    public static final Mode currentMode = Mode.REAL;
    public static final boolean tuningMode = true;
    public static final LoggedDashboardNumber deadzone = new LoggedDashboardNumber("CONTROLLER_DEADZONE", 0.1);
    
    public static final double deadzone(double input) {
        return Math.abs(input) > deadzone.get() ? input : 0;
    }

    public static final class ShooterConstants {
		public static LoggedDashboardNumber SPEAKER_ANGLE = new LoggedDashboardNumber("Shooter/Constants/SPEAKER_ANGLE", 0.852);
		public static LoggedDashboardNumber SPEAKER_SPEED = new LoggedDashboardNumber("Shooter/Constants/SPEAKER_SPEED",3100);
        public static LoggedDashboardNumber SPEAKER_ROLLER_VOLTAGE = new LoggedDashboardNumber("Shooter/Constants/SPEAKER_ROLLER_VOLTAGE",12);

		public static LoggedDashboardNumber AMP_ANGLE = new LoggedDashboardNumber("Shooter/Constants/AMP_ANGLE",0.812);
		public static LoggedDashboardNumber AMP_SPEED = new LoggedDashboardNumber("Shooter/Constants/AMP_SPEED",430);
        public static LoggedDashboardNumber AMP_ROLLER_VOLTAGE = new LoggedDashboardNumber("Shooter/Constants/AMP_ROLLER_VOLTAGE", AMP_SPEED.get()/700*12);

		public static LoggedDashboardNumber FLYWHEEL_SPEED_P = new LoggedDashboardNumber("Shooter/Constants/FLYWHEEL_SPEED_P",0.0005);
        public static LoggedDashboardNumber FLYWHEEL_SPEED_I = new LoggedDashboardNumber("Shooter/Constants/FLYWHEEL_SPEED_I",0);
		public static LoggedDashboardNumber FLYWHEEL_SPEED_D = new LoggedDashboardNumber("Shooter/Constants/FLYWHEEL_SPEED_D",0.0005);
		public static LoggedDashboardNumber FLYWHEEL_SPEED_FF = new LoggedDashboardNumber("Shooter/Constants/FLYWHEEL_SPEED_FF",0.00024);

        public static LoggedDashboardNumber PIVOT_P = new LoggedDashboardNumber("Shooter/Constants/PIVOT_P",10);
        public static LoggedDashboardNumber PIVOT_I = new LoggedDashboardNumber("Shooter/Constants/PIVOT_I",0);
        public static LoggedDashboardNumber PIVOT_D = new LoggedDashboardNumber("Shooter/Constants/PIVOT_D",0);
        public static LoggedDashboardNumber MIN_PIVOT_ANGLE = new LoggedDashboardNumber("Shooter/Constants/MIN_PIVOT_ANGLE",0.81);
        public static LoggedDashboardNumber MAX_PIVOT_ANGLE = new LoggedDashboardNumber("Shooter/Constants/MAX_PIVOT_ANGLE",0.99);
    }

}
