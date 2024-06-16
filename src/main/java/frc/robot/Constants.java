package frc.robot;

public final class Constants {
    public static enum Mode {
        REAL,
        SIM,
        REPLAY
    }

    public static final Mode currentMode = Mode.SIM;

    public static final class ClimberConstants {
        public static final double WINCH_P = .00005;
        public static final double WINCH_I = .00005;
        public static final double WINCH_D = .00005;
        public static final double WINCH_FF = .00005; 
    }
}
