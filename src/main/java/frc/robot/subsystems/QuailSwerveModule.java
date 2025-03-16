package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.mineinjava.quail.SwerveModuleBase;
import com.mineinjava.quail.util.geometry.Vec2d;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.math.Constants;

public class QuailSwerveModule extends SwerveModuleBase {

	/** The motor controlling the module's movement. */
	private final TalonFX drivingMotor;
	/** The motor controlling the module's rotation. */
	private final TalonFX steeringMotor;
	/** The can coder measuring the module's absolute rotaiton. */
	private final CANcoder canCoder;
	/**
	 * The can coder's rotational offset. This value must be manually set through
	 * phoenix tuner.
	 */
	private final double canOffset;

	private int steeringMotorID;

	private int resets;

	public QuailSwerveModule(Vec2d position, int driveMotorID, int steeringMotorID, int canCoderID,
			double canCoderOffset) {

		super(position, steeringMotorID, canCoderOffset, true);
		this.drivingMotor = new TalonFX(driveMotorID);
		this.steeringMotor = new TalonFX(steeringMotorID);
		this.canCoder = new CANcoder(canCoderID);
		this.canOffset = canCoderOffset;

		this.steeringMotorID = steeringMotorID;

		resets = 0;
	}

	public void init() {

		System.out.println("Initializing Swerve modue [sid: ]" + this.steeringMotorID);
		// Reset the steering motor.
		MotorOutputConfigs motorConfig = new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive);

		TalonFXConfiguration driveTalonConfig = new TalonFXConfiguration().withMotorOutput(motorConfig);

		TalonFXConfiguration steeringTalonConfig = new TalonFXConfiguration().withMotorOutput(motorConfig);

		// PID tunes the steering motor
		steeringTalonConfig.Slot0.kV = Constants.PID_SETTINGS[0];
		steeringTalonConfig.Slot0.kP = Constants.PID_SETTINGS[1];
		steeringTalonConfig.Slot0.kI = Constants.PID_SETTINGS[2];

		this.steeringMotor.getConfigurator().apply(steeringTalonConfig);
		this.drivingMotor.getConfigurator().apply(driveTalonConfig);

		MagnetSensorConfigs encoderConfig = new MagnetSensorConfigs();
		encoderConfig.AbsoluteSensorDiscontinuityPoint = 1;
		encoderConfig.MagnetOffset = this.canOffset;
		encoderConfig.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

		this.canCoder.getConfigurator().apply(encoderConfig);
	}

	/**
	 * 1. Gets Encoder Angle
	 * 2. Spins the motor so the encoder's 0
	 * 3. Sets the motor's position to 0
	 */
	public void reset() {
		System.out.println("Resetting steering module ID: " + this.steeringMotorID);
		this.steeringMotor.setPosition(
			this.getRawAngle() * Constants.STEERING_RATIO
		);
		this.currentAngle = this.getRawAngle() * Math.PI * 2;
		this.drivingMotor.stopMotor();
		this.steeringMotor.stopMotor();
	}

	/*
	 * public Vec2d getCurrentMovement() {
	 * double angle = this.canCoder.getAbsolutePosition().refresh().getValue() * (2
	 * * Math.PI);
	 * double velocity = this.drivingMotor.getVelocity().refresh().getValue();
	 * double rotationsPerSecond = velocity / 6.75;
	 * double inchesPerSecond = rotationsPerSecond * Constants.WHEEL_DIAMETER *
	 * Math.PI;
	 * SmartDashboard.putNumber("Module" + this.canCoder.getDeviceID(),
	 * inchesPerSecond);
	 * 
	 * return new Vec2d(angle, inchesPerSecond, false);
	 * 
	 * }
	 */

	/**
	 * Input in radians
	 * Create an angle object
	 * Set position
	 */

	@Override
	public void setRawAngle(double angleInRad) {
		Angle angle = Angle.ofBaseUnits(angleInRad, Radian);
		this.steeringMotor.setControl(new PositionDutyCycle(angle.times(Constants.STEERING_RATIO)));
		SmartDashboard.putNumber("Module " + steeringMotorID + " target angle: ", angle.in(Rotations));
	}

	// Returns the position of the canencoder attached to this module
	public double getRawAngle() {
		double currentPos = this.canCoder.getAbsolutePosition().refresh().getValue().in(Rotation);
		currentPos = (currentPos + 1) % 1;
		return currentPos;
	}

	public double meep() {
		return this.canCoder.getAbsolutePosition().refresh().getValue().in(Rotation);
	}

	// Returns the position of the steering motor
	public Angle getRotations() {
		return (this.steeringMotor.getPosition().refresh().getValue().div(Constants.STEERING_RATIO));
	}


	@Override
	public void setRawSpeed(double speed) {
		this.drivingMotor.set(speed);
	}

	public void setRawSteeringSpeed(double speed) {
		this.steeringMotor.set(speed);
	}

	public void setBrake(ControlRequest brake) {
		this.drivingMotor.setControl(brake);
	}

	/*
	 * public TalonFX getDriveMotor()
	 * {
	 * return this.drivingMotor;
	 * }
	 * 
	 * 
	 * public TalonFX getSteeringMotor()
	 * {
	 * return this.steeringMotor;
	 * }
	 * 
	 * 
	 * public CANcoder getCanCoder()
	 * {
	 * return this.canCoder;
	 * }
	 */

	@Override
	public String toString() {
		// The class will be represented as "SwerveModule[Steering Motor ID = ?, Driving
		// Motor ID = ?, Cancoder ID = ?]"
		return "SwerveModule[Steering Motor ID = " + this.steeringMotor.getDeviceID() + ", Driving Motor ID = "
				+ this.drivingMotor.getDeviceID() + ", Cancoder ID = " + this.canCoder.getDeviceID() + "]";
	}
}
