package frc.robot.math;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;

public class Constants {
	/*
	 * AUTHOR'S NOTE
	 * 
	 * For all degree/radians/rotation unit conversions (ex. radians to degrees),
	 * use Angle objects
	 * 
	 * angle.in(units)
	 * Angle.ofBaseUnits();
	 * 
	 * 
	 */

	/**
	 * The delay in milliseconds before a report gets sent to DriverStation if an
	 * action fails.
	 */
	// public static final int MS_DELAY = 30;

	// The port of the driver controller
	public static final int DRIVER_PORT = 0;

	// The port of the operator (secondary driver) controller
	public static final int OPERATOR_PORT = 1;

	// Diameter of colson swerve wheels
	public static final double WHEEL_DIAMETER = 4;

	public static final double STEERING_RATIO = 12.8d;

	public static final double DRIVE_RATIO = 6.75d;

	/** Settings are {kF, kP, kI, kD} */// kV kP kI kD
	// Mainly pay attention to kP
	public static final double[] PID_SETTINGS = { 0.0d, 0.5d, 0.0d, 0.00d };

	/** 2π */
	public static final double TWO_PI = 2d * Math.PI;

	/** π/2 */
	public static final double PI_OVER_TWO = Math.PI / 2d;

	/** Angle precision for rotation */
	public static final double ANGLE_PRECISION = Math.PI / 32;

	public static final int INCHES_PER_FOOT = 12;

	public static final double METERS_TO_INCHES = 39.3701;

	// How far the shaft of the swerve module is away from the exact center of
	// rotation
	public static final double CENTER_TO_SWERVE_DIST = 10.75d;

	// The dead zone of the joystick
	// (The zone where we round down to 0 to combat stick drift)
	public static final double JOYSTICK_DEAD_ZONE = 0.1d;

	// Speed without the trigger
	public static final double BASE_SPEED = 0.08d;

	// The slowest speed the robot can go
	public static final double MINIMUM_SPEED = 0.05d;

	// The inverse constant of the rotation speed scale
	public static final double ROTATION_SPEED_INVERSE_SCALE = 35;

	// Limits change in the left joystick
	public static final double MOVEMENT_ACCELERATION_LIMIT = 0.1d;

	// Limits change in the right joystick
	public static final double ROTATION_ACCELERATION_LIMIT = 0.1d;

	// Limits change in the speed trigger
	public static final double SPEED_ACCELERATION_LIMIT = 0.1d;

	// Limits change in the overall movement of the robot
	public static final double DRIVE_ACCELERATION_LIMIT = 0.003d;

	// The IDs of the drive motors
	public static final int[] DRIVE_MOTOR_IDS = new int[] {
			1,
			3,
			5,
			7
	};

	// The IDs of the drive motors
	public static final int[] STEER_MOTOR_IDS = new int[] {
			2,
			4,
			6,
			8
	};

	// The CAN IDs of the canCoders
	public static final int[] CANCODER_IDS = new int[] {
			21,
			22,
			23,
			24
	};

	// The CAN IDs of the Elevator motors
	public static final int[] ELEVATOR_IDS = new int[] {
			21,
			22
	};

	public static final int ARM_MOTOR_ID = 31;

	public static final int CLIMBER_MOTOR_ID = 41;

	// Individual canCoder offsets that are determined by the Phoenix Tuner X
	public static final double[] CANCODER_OFFSETS = new double[] {
			-.83,
			-.92,
			-.52,
			-.27
	};

	// The offset of the roboRio gyro
	// Factored in when moving the entire drivetrain
	public static final Angle GYRO_OFFSET = Angle.ofBaseUnits(-PI_OVER_TWO, Radian);

	// The offset of each individual encoder
	// Factored in when rotating each steering motor

	//////////// ELEVATOR / ARM Constants

	public static final double ELEVATOR_MANUAL_SPEED = 0.1;

	// Bounds for the elevator in rotations
	public static final double ELEVATOR_MIN_ROTATIONS = 0d;
	public static final double ELEVATOR_MAX_ROTATIONS = 41d;

	public static final double ELEVATOR_MAX_SPEED = 0.2d;
	public static final double ARM_MAX_SPEED = 0.2d;

	public static final double ARM_MAX_ROTATIONS = 0.25;
	public static final double ARM_MIN_ROTATIONS = -0.25;

	public static final double ARM_GEAR_RATIO = 81;

	public static final double ARM_TOLERANCE = 0.05;
	public static final double ELEVATOR_TOLERANCE = 0.3;

	public static final double ELEVATOR_L1 = 0;
	public static final double ELEVATOR_L2 = 9;
	public static final double ELEVATOR_L3 = 22;
	public static final double ELEVATOR_L4 = 41;

	public static final double ELEVATOR_STOW = 0;
	public static final double ELEVATOR_TRANSITION = 14;
	public static final double ELEVATOR_PICKUP = 10;

	public static final double ARM_STOW = 0.25;

	public static final double ARM_OUT = 0;
	public static final double ARM_DOWN = -0.25;
	public static final double ARM_SCORE_HIGH = 0.17;
	public static final double ARM_SCORE_LOW = 0.15;

	public static final double ARM_UPPER_ELEVATOR_CLEARANCE = 0.2;
	public static final double ARM_LOWER_ELEVATOR_CLEARANCE = -0.05;

	public static final double ELEVATOR_TIME_TO_MAX_SPEED = 1;
	public static final double ARM_TIME_TO_MAX_SPEED = 1;

    public static final double CLIMBER_SPEED = 0.2;
}
