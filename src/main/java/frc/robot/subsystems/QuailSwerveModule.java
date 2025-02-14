package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.mineinjava.quail.SwerveDrive;
import com.mineinjava.quail.SwerveModuleBase;
import com.mineinjava.quail.util.geometry.Vec2d;

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
	/** The can coder's rotational offset. This value must be manually set through phoenix tuner. */
	private final double canOffset;

	private int steeringMotorID;

	private int resets;

    public QuailSwerveModule(Vec2d position, int driveMotorID, int steeringMotorID, int canCoderID, double canCoderOffset){

        super(position, steeringMotorID, canCoderOffset, true);
        this.drivingMotor = new TalonFX(driveMotorID);
		this.steeringMotor = new TalonFX(steeringMotorID);
		this.canCoder = new CANcoder(canCoderID);
		this.canOffset = canCoderOffset;

		this.steeringMotorID = steeringMotorID;
		
		resets = 0;
    }

    public void init()
    {

		System.out.println("Initializing Module");
		// Reset the steering motor.
        MotorOutputConfigs motorConfig = new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive);


		TalonFXConfiguration driveTalonConfig = new TalonFXConfiguration().withMotorOutput(motorConfig);

		TalonFXConfiguration steeringTalonConfig = new TalonFXConfiguration().withMotorOutput(motorConfig);

		//PID tunes the steering motor
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
	
		// Miscellaneous settings.
		
		
		/*var slot0Configs = new Slot0Configs();
		slot0Configs.kS = 0.0; // Add 0.25 V output to overcome static friction
		slot0Configs.kA = 0.00; // An acceleration of 1 rps/s requires 0.01 V output
		slot0Configs.kV = Constants.PID_SETTINGS[0];
		slot0Configs.kP = Constants.PID_SETTINGS[1];
		slot0Configs.kI = Constants.PID_SETTINGS[2];
		slot0Configs.kD = Constants.PID_SETTINGS[3];
        

		var motionMagicConfigs = talonFXConfigs.MotionMagic;
		motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
		motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
		motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

		this.steeringMotor.getConfigurator().apply(talonFXConfigs);
	
		// PID tune the steering motor.

		this.steeringMotor.getConfigurator().apply(slot0Configs);

    	System.out.println("Done Config");
        */

		// Reset the motor rotations.
		this.reset();
		
	}

    

	/**
	 * 1. Gets Encoder Angle
     * 2. Spins the motor so the encoder's 0
     * 3. Sets the motor's position to 0
	 */
	public void reset()
	{
		System.out.println("RESET");
        //Angle currentPos = this.canCoder.getAbsolutePosition(true).getValue();
        //this.steeringMotor.setControl(new PositionDutyCycle(currentPos.unaryMinus().times(Constants.STEERING_RATIO)));
        //this.steeringMotor.setPosition(0);
		
		this.drivingMotor.stopMotor();
		this.steeringMotor.stopMotor();
	}

	/*public Vec2d getCurrentMovement() {
		double angle = this.canCoder.getAbsolutePosition().refresh().getValue() * (2 * Math.PI);
		double velocity = this.drivingMotor.getVelocity().refresh().getValue();
		double rotationsPerSecond = velocity / 6.75;
		double inchesPerSecond = rotationsPerSecond * Constants.WHEEL_DIAMETER * Math.PI;
		SmartDashboard.putNumber("Module" + this.canCoder.getDeviceID(), inchesPerSecond);

		return new Vec2d(angle, inchesPerSecond, false);
	
	}*/
	
	/**
	 * Input in radians
     * Create an angle object
     * Set position
	 */

	@Override
	public void setRawAngle(double angleInRad)
	{

        Angle angle = Angle.ofBaseUnits(angleInRad, Radian);
		Angle trueAngle = angle.plus(Constants.ENCODER_OFFSET);
        this.steeringMotor.setControl(new PositionDutyCycle(trueAngle.times(Constants.STEERING_RATIO)));

		System.out.println("Module " + steeringMotorID + ": " + angle.in(Degree));

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
     	public TalonFX getDriveMotor()
	{
		return this.drivingMotor;
	}


	public TalonFX getSteeringMotor()
	{
		return this.steeringMotor;
	}

	
	public CANcoder getCanCoder()
	{
		return this.canCoder;
	}
    */

	@Override
	public String toString()
	{
		// The class will be represented as "SwerveModule[Steering Motor ID = ?, Driving Motor ID = ?, Cancoder ID = ?]"
		return "SwerveModule[Steering Motor ID = " + this.steeringMotor.getDeviceID() + ", Driving Motor ID = " + this.drivingMotor.getDeviceID() + ", Cancoder ID = " + this.canCoder.getDeviceID() + "]";
	}
}
