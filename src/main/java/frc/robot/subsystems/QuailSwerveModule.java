package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MusicTone;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.mineinjava.quail.SwerveModuleBase;
import com.mineinjava.quail.util.geometry.Vec2d;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
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

  private Angle angle;
  private double speed;

  public QuailSwerveModule(
      Vec2d position,
      int driveMotorID,
      int steeringMotorID,
      int canCoderID,
      double canCoderOffset) {

    super(position, steeringMotorID, canCoderOffset, true);
    this.drivingMotor = new TalonFX(driveMotorID);
    this.steeringMotor = new TalonFX(steeringMotorID);
    this.canCoder = new CANcoder(canCoderID);
    this.canOffset = canCoderOffset;

    this.steeringMotorID = steeringMotorID;
  }

  public void init() {

    System.out.println("Initializing Swerve modue [sid: ]" + this.steeringMotorID);
    // Reset the steering motor.
    MotorOutputConfigs motorConfig = new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive);

    TalonFXConfiguration driveTalonConfig = new TalonFXConfiguration().withMotorOutput(motorConfig);

    TalonFXConfiguration steeringTalonConfig = new TalonFXConfiguration().withMotorOutput(motorConfig);

    driveTalonConfig.Audio.withAllowMusicDurDisable(true);
    steeringTalonConfig.Audio.withAllowMusicDurDisable(true);

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

    this.angle = getRawAngle();
    this.speed = 0;
    this.reset();
  }

  /**
   * 1. Gets Encoder Angle 2. Spins the motor so the encoder is at 0 3. Sets the
   * motor's position to
   * 0
   */
  public void reset() {
    System.out.println("Resetting steering module ID: " + this.steeringMotorID);
    this.steeringMotor.setPosition(this.getRawAngle().in(Rotation) * Constants.STEERING_RATIO);
    this.currentAngle = this.getRawAngle().in(Radian);
    this.drivingMotor.stopMotor();
    this.steeringMotor.stopMotor();
  }

  /** Input in radians Create an angle object Set position */
  @Override
  public void setRawAngle(double angleInRad) {
    angle = Angle.ofBaseUnits(angleInRad, Radian);
    this.steeringMotor.setControl(new PositionDutyCycle(angle.times(Constants.STEERING_RATIO)));
  }

  /**
   * @return angle in rotations of the encoder, not bounded
   */
  public Angle getRawAngle() {
    double currentPos = this.canCoder.getAbsolutePosition().refresh().getValue().in(Rotation);
    return Angle.ofBaseUnits(currentPos, Rotation);
  }

  /**
   * @return the true angle of the encoder in rotations, bounded 0 to 1
   */
  public Angle getNormalizedAngle() {
    double currentPos = this.canCoder.getAbsolutePosition().refresh().getValue().in(Rotation);
    // Normalizes angle
    currentPos = (currentPos + 1) % 1;
    return Angle.ofBaseUnits(currentPos, Rotation);
  }

  /**
   * @return the true angle of the motor in rotations, not bounded
   */
  public Angle getTrueAngle() {
    return (this.steeringMotor.getPosition().refresh().getValue().div(Constants.STEERING_RATIO));
  }

  /**
   * @return Desired angle of the module, bounded between 0 and 2π
   */
  public Angle getDesiredAngleNormalized() {
    double angleInRad = MathUtil.angleModulus(angle.in(Radian)) + Math.PI;
    return Angle.ofBaseUnits(angleInRad, Radian);
  }

  @Override
  public void setRawSpeed(double speed) {
    this.speed = speed;
    this.drivingMotor.set(speed);
  }

  public double getDesiredSpeed() {
    return this.speed;
  }

  public void setBrake(ControlRequest brake) {
    this.drivingMotor.setControl(brake);
  }

  public void setMotorSound(int frequency) {
    this.drivingMotor.setControl(new MusicTone(frequency));
    this.steeringMotor.setControl(new MusicTone(frequency));
  }

  public TalonFX getDriveMotor() {
    return this.drivingMotor;
  }

  public TalonFX getSteerMotor() {
    return this.steeringMotor;
  }

  @Override
  public String toString() {
    // The class will be represented as "SwerveModule[Steering Motor ID = ?, Driving
    // Motor ID = ?, Cancoder ID = ?]"
    return "SwerveModule[Steering Motor ID = "
        + this.steeringMotor.getDeviceID()
        + ", Driving Motor ID = "
        + this.drivingMotor.getDeviceID()
        + ", Cancoder ID = "
        + this.canCoder.getDeviceID()
        + "]";
  }

  /**
   * @return The desired state of the module, based on passed in values
   */
  public SwerveModuleState getDesiredState() {
    Rotation2d r = new Rotation2d(this.angle.in(Radian));
    return new SwerveModuleState(this.speed, r);
  }

  /**
   * @return The actual state of the module, based on the can coder and driving
   *         motor
   *         NOTE: speed is based on last set speed, not actual speed
   */
  public SwerveModuleState getActualState() {
    Rotation2d r = new Rotation2d(getNormalizedAngle().in(Radian));
    return new SwerveModuleState(this.speed, r);
  }

  public Vec2d getCurrentMovement() {
    return new Vec2d(
        this.canCoder.getAbsolutePosition().getValueAsDouble() * Math.PI * 2,
        this.drivingMotor.getVelocity().refresh().getValueAsDouble()
            * Math.PI
            * Constants.WHEEL_DIAMETER
            / Constants.DRIVE_RATIO,
        false);
  }
}
