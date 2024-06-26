package frc.robot;

public final class Ports {
    public static final class ShooterPorts {
        public static final int leftFlywheel = 32;  //CAN
        public static final int rightFlywheel = 33; //CAN
        public static final int pivot = 34;         //CAN
        public static final int pivotEncoder = 2;   //DIO
        public static final int rollerTop = 30;     //CAN
        public static final int rollerBottom = 31;  //CAN
    }

    public static final class IntakePorts {
        public static final int runInternal = 40;
        public static final int runExternal = 41;
        public static final int beamBreak = 6;
    }
}
