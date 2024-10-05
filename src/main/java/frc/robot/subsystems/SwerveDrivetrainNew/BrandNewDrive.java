package frc.robot.subsystems.SwerveDrivetrainNew;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BrandNewDrive extends SubsystemBase {
    
    private static final double FRONT_LEFT_VIRTUAL_OFFSET_RADIANS = -3.03+Math.PI; // adjust as needed so that virtual (turn) position of wheel is zero when straight
	private static final double FRONT_RIGHT_VIRTUAL_OFFSET_RADIANS = 2.69; // adjust as needed so that virtual (turn) position of wheel is zero when straight
	private static final double REAR_LEFT_VIRTUAL_OFFSET_RADIANS = -2.33; // adjust as needed so that virtual (turn) position of wheel is zero when straight
	private static final double REAR_RIGHT_VIRTUAL_OFFSET_RADIANS = Math.PI/2+Math.PI/16-Math.PI; // adjust as needed so that virtual (turn) position of wheel is zero when straight
    private static final int GYRO_ORIENTATION = 1; // might be able to merge with kGyroReversed

    private static final double FIELD_LENGTH_INCHES = 54*12+1; // 54ft 1in
	private static final double FIELD_WIDTH_INCHES = 26*12+7; // 26ft 7in

    /** TURN SETTINGS */
	// NOTE: it might make sense to decrease the PID controller period below 0.02 sec (which is the period used by the main loop)
	private static final double TURN_PID_CONTROLLER_PERIOD_SECONDS = .02; // 0.01 sec = 10 ms 	
	
	private static final double MIN_TURN_PCT_OUTPUT = 0.1; // 0.1;
	private static final double MAX_TURN_PCT_OUTPUT = 0.4; // 0.4;
	
	private static final double TURN_PROPORTIONAL_GAIN = 0.001; // 0.01;
	private static final double TURN_INTEGRAL_GAIN = 0.0;
	private static final double TURN_DERIVATIVE_GAIN = 0.0; // 0.0001
	
	private static final int DEGREE_THRESHOLD = 10; // 3;
	
	private final static int TURN_ON_TARGET_MINIMUM_COUNT = 10; // number of times/iterations we need to be on target to really be on target
	/** END TURN SETTINGS */

    public enum DriveState {
        MANUAL,
        FOLLOW_PATH,
        SPEAKER_AUTO_ALIGN,
        X,
        TUNING
    }

    @Override
    public void periodic() {

    }
    
}
