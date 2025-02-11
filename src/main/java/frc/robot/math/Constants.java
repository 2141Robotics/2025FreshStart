package frc.robot.math;

public class Constants {
	/*AUTHOR'S NOTE

	For all degree/radians/rotation unit conversions (ex. radians to degrees), use:
	
	Angle.ofBaseUnits();

	*/


    /** The delay in milliseconds before a report gets sent to DriverStation if an action fails. */
	//public static final int MS_DELAY = 30;

	//The port of the driver controller
	public static final int DRIVER_PORT = 0;
	
	//The port of the operator (secondary driver) controller
	public static final int OPERATOR_PORT = 1;

	//Diameter of colson swerve wheels
	public static final double WHEEL_DIAMETER = 4;

	public static final double STEERING_RATIO = 12.8d;

	public static final double DRIVE_RATIO = 6.75d;

	/** Settings are {kF, kP, kI, kD} *///		kV  kP	 kI  kD
	//Mainly pay attention to kP
	public static final double[] PID_SETTINGS =  { 0.0d, 0.5d, 0.0d, 0.00d };

	/** 2π */
	public static final double TWO_PI = 2d * Math.PI;

	/** π/2 */
	public static final double PI_OVER_TWO = Math.PI / 2d;

	/** Angle precision for rotation */
	public static final double ANGLE_PRECISION = Math.PI / 32;

	public static final int INCHES_PER_FOOT = 12;

	public static final double METERS_TO_INCHES = 39.3701;

	//How far the shaft of the swerve module is away from the exact center of rotation
	public static final double CENTER_TO_SWERVE_DIST = 10.75d;

	//The dead zone of the joystick
	//(The zone where we round down to 0 to combat stick drift)
    public static final double JOYSTICK_DEAD_ZONE = 0.1;

	//Speed without the trigger
	public static final double BASE_SPEED = 0.08;

	//*Shrug*
	public static final double ROTATION_SPEED_INVERSE_SCALE = 35;

	//Limits change in the left joystick
	public static final double MOVEMENT_ACCELERATION_LIMIT = 0.1d;

	//Limits change in the right joystick
    public static final double ROTATION_ACCELERATION_LIMIT = 0.1d;

	//Limits change in the speed trigger
    public static final double SPEED_ACCELERATION_LIMIT = 0.1d;

	//Limits change in the overall movement of the robot
	public static final double DRIVE_ACCELERATION_LIMIT = 0.003d;

	public static final double[] CANCODER_OFFSETS = new double[] {
		-0.08154296875,
		-0.16357421875,
		-0.761962890625,
		0.471923828125
	};

	public static final int[] CANCODER_IDS = new int[]{
		21,
		22,
		23,
		24
	};
}
