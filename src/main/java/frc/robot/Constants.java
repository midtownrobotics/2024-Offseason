package frc.robot;

public final class Constants {
    public static enum Mode {
        REAL,
        SIM,
        REPLAY
    }

    public static final Mode currentMode = Mode.SIM;
}
