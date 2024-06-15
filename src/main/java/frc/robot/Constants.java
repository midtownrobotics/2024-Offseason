package frc.robot;

public final class Constants {
    public static enum Mode {
        REAL,
        SIM,
        REPLAY
    }

    public static final Mode currentMode = Mode.SIM;

    public static final class ShooterConstants {
		public static final double SPEAKER_ANGLE = 0.852;
		public static final double SPEAKER_SPEED = 3100;
		public static final double AMP_ANGLE = 0.812;
		public static final double AMP_SPEED = 430;
		public static final double LOW_ANGLE = 0.98;
		public static final double FLYWHEEL_SPEED_P = 0.0005;
        public static final double FLYWHEEL_SPEED_I = 0;
		public static final double FLYWHEEL_SPEED_D = 0.0005;
		public static final double FLYWHEEL_SPEED_FF = 0.00024;
    }
}
