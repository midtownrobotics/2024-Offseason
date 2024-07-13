package frc.robot;

public final class Ports {

    public static final int driverControllerPort = 0;
    public static final int operatorControllerPort = 1;

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
    
    public static final class ClimberPorts {
        public static final int leftClimberID = 50;
        public static final int rightClimberID = 51;
    }


    public static final class NeoDrive {
        public static final int FRONT_RIGHT_DRIVING = 13;
        public static final int FRONT_RIGHT_TURNING = 14;
        public static final int FRONT_RIGHT_TURNING_ABSOLUTE_ENCODER = 15;			

        public static final int FRONT_LEFT_DRIVING = 10;
        public static final int FRONT_LEFT_TURNING = 11;
        public static final int FRONT_LEFT_TURNING_ABSOLUTE_ENCODER = 12;
        
        public static final int REAR_LEFT_DRIVING = 19;
        public static final int REAR_LEFT_TURNING = 20;
        public static final int REAR_LEFT_TURNING_ABSOLUTE_ENCODER = 21;    

        public static final int REAR_RIGHT_DRIVING = 16;
        public static final int REAR_RIGHT_TURNING = 17;
        public static final int REAR_RIGHT_TURNING_ABSOLUTE_ENCODER = 18;
    }
}
