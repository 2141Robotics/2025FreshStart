// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.StaticBrake;
import com.mineinjava.quail.RobotMovement;
import com.mineinjava.quail.localization.KalmanFilterLocalizer;
import com.mineinjava.quail.localization.SwerveOdometry;
import com.mineinjava.quail.util.MiniPID;
import com.mineinjava.quail.util.geometry.Pose2d;
import com.mineinjava.quail.util.geometry.Vec2d;
import com.studica.frc.AHRS;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.math.Constants;
import java.util.ArrayList;

public class Drivetrain extends SubsystemBase {

  private AHRS gyro;

  private QuailSwerveDrive quailSwerveDrive;

  private ArrayList<QuailSwerveModule> modules;

  public Boolean HAS_LIMELIGHT = false;

  public SwerveOdometry odometry;
  public MiniPID pidcontroller;
  private KalmanFilterLocalizer kalmanFilter =
      new KalmanFilterLocalizer(new Pose2d(0, 0, 0), Constants.LOOPTIME);

  /** Creates a new ExampleSubsystem. */
  public Drivetrain(AHRS gyro) {

    this.gyro = gyro;

    modules = new ArrayList<>();

    modules.add(
        new QuailSwerveModule(
            new Vec2d(Constants.CENTER_TO_SWERVE_DIST, Constants.CENTER_TO_SWERVE_DIST),
            Constants.DRIVE_MOTOR_IDS[0],
            Constants.STEER_MOTOR_IDS[0],
            Constants.CANCODER_IDS[0],
            Constants.CANCODER_OFFSETS[0]));
    modules.add(
        new QuailSwerveModule(
            new Vec2d(Constants.CENTER_TO_SWERVE_DIST, -Constants.CENTER_TO_SWERVE_DIST),
            Constants.DRIVE_MOTOR_IDS[1],
            Constants.STEER_MOTOR_IDS[1],
            Constants.CANCODER_IDS[1],
            Constants.CANCODER_OFFSETS[1]));
    modules.add(
        new QuailSwerveModule(
            new Vec2d(-Constants.CENTER_TO_SWERVE_DIST, -Constants.CENTER_TO_SWERVE_DIST),
            Constants.DRIVE_MOTOR_IDS[2],
            Constants.STEER_MOTOR_IDS[2],
            Constants.CANCODER_IDS[2],
            Constants.CANCODER_OFFSETS[2]));
    modules.add(
        new QuailSwerveModule(
            new Vec2d(-Constants.CENTER_TO_SWERVE_DIST, Constants.CENTER_TO_SWERVE_DIST),
            Constants.DRIVE_MOTOR_IDS[3],
            Constants.STEER_MOTOR_IDS[3],
            Constants.CANCODER_IDS[3],
            Constants.CANCODER_OFFSETS[3]));

    this.quailSwerveDrive = new QuailSwerveDrive(modules);
    this.odometry = new SwerveOdometry(quailSwerveDrive);
  }

  /**
   * Checks if the robot is resetting or if the gyro is callibrating.
   *
   * @return Whether or not the robot can drive
   */
  public boolean canDrive() {
    return !this.gyro.isCalibrating();
  }

  public void reset() {
    quailSwerveDrive.reset();
    resetGyro();
    setMotorBreaking(new StaticBrake());
    System.out.println("RESET DRIVETRAIN");
  }

  public void init() {
    quailSwerveDrive.initModules();
    resetGyro();
    reset();
  }

  public void drive(RobotMovement robotMovement) {
    if (canDrive()) {
      quailSwerveDrive.drive(robotMovement, this.gyro.getAngle());
      // System.out.println("GYRO " + this.gyro.getAngle());
    } else {
      System.out.println("cannot drive, calibrating gyro");
    }
  }

  public Command xLockCommand(){
    return this.runOnce(
        () -> {
          this.xLock();
        });
  }

  public Command disableMotorBreakingCommand(){
    return this.runOnce(
        () -> {
          this.setMotorBreaking(new CoastOut());
        });
  }

  public Command enableMotorBreakingCommand(){
    return this.runOnce(
        () -> {
          this.setMotorBreaking(new StaticBrake());
        });
  }

  public void setMotorBreaking(ControlRequest controlRequest) {
    quailSwerveDrive.setBrake(controlRequest);
  }

  public void xLock() {
    quailSwerveDrive.XLock();
  }

  public void stop() {
    quailSwerveDrive.stop();
  }

  public SwerveOdometry getOdometry() {
    return this.odometry;
  }

  public KalmanFilterLocalizer getKalmanFilter() {
    return this.kalmanFilter;
  }

  public Angle getGyroAngle() {
    Angle angle = Angle.ofBaseUnits(this.gyro.getAngle(), Radians);
    // System.out.println("Gyro Angle: " + this.gyro.getAngle());
    // System.out.println("Angle Object Value: " + angle.in(Radians));
    return angle;
  }

  /** Reset the gyro to 0°. */
  public void resetGyro() {
    this.gyro.reset();
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command ExampleCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  public Command resetGyroCommand() {
    return this.runOnce(() -> this.resetGyro());
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    super.periodic();
    // This method will be called once per scheduler run
    for (int i = 0; i < 4; i++) {
      SmartDashboard.putNumber(
          "Module " + (i + 1) + " raw angle:", this.modules.get(i).getRawAngle());
      SmartDashboard.putNumber("Module " + (i + 1) + " meep:", this.modules.get(i).meep());
      SmartDashboard.putNumber(
          "Module " + (i + 1) + " angle:", this.modules.get(i).getRotations().in(Rotation));
    }
    this.updateOdometry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }

  public void updateOdometry() {
    ArrayList<Vec2d> moduleSpeeds = this.quailSwerveDrive.getModuleSpeeds();
    RobotMovement velocity = this.odometry.calculateFastOdometry(moduleSpeeds);
    //System.out.println("velocity: "+ velocity.translation.getLength());

    this.odometry.updateDeltaPoseEstimate(velocity.translation.scale(0.02));
    this.odometry.setAngle(this.gyro.getAngle() * Math.PI * 2);

    double[] LL2pos =
        NetworkTableInstance.getDefault()
            .getTable("limelight-two")
            .getEntry("botpose")
            .getDoubleArray(new double[6]);

    double[] LL3pos =
        NetworkTableInstance.getDefault()
            .getTable("limelight-three")
            .getEntry("botpose")
            .getDoubleArray(new double[6]);

    SmartDashboard.putNumberArray("Limelight 2 Pos", LL2pos);
    SmartDashboard.putNumberArray("Limelight 3 Pos", LL3pos);

    double LX = 0;
    double LY = 0;
    double LATENCY = 0;

    if (LL3pos.length >= 7) {
      LX = LL3pos[1] * Constants.INCHES_PER_METER;
      LY = -LL3pos[0] * Constants.INCHES_PER_METER;
      LATENCY = LL3pos[6];
      if (LL3pos[0] != 0) {
        NetworkTableInstance.getDefault()
            .getTable("limelight-three")
            .getEntry("ledMode")
            .setNumber(3);
      } else {
        NetworkTableInstance.getDefault()
            .getTable("limelight-three")
            .getEntry("ledMode")
            .setNumber(1);
      }
    }
    if (LL2pos.length >= 7) {
      if (true) {
        LX = LL2pos[1] * Constants.INCHES_PER_METER;
        LY = -LL2pos[0] * Constants.INCHES_PER_METER;
        LATENCY = LL2pos[6];
      }
      if (LL2pos[0] != 0) {
        NetworkTableInstance.getDefault()
            .getTable("limelight-two")
            .getEntry("ledMode")
            .setNumber(3);
      } else {
        NetworkTableInstance.getDefault()
            .getTable("limelight-two")
            .getEntry("ledMode")
            .setNumber(1);
      }
    }

    if (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue) {
      LY = -LY; // invert y
      LX = -LX;
    }

    SmartDashboard.putNumber("LX", LX);
    SmartDashboard.putNumber("LY", LY);

    double w = Constants.KALMAN_FILTER_W;
    if ((LX == 0) && (LY == 0)) {
      w = 0;
      this.HAS_LIMELIGHT = false;
    } else {
      this.HAS_LIMELIGHT = true;
    }

    this.kalmanFilter.update(
        new Pose2d(LX, LY, 0),
        new Pose2d(velocity.translation.rotate(-this.gyro.getAngle(), true).rotate(Math.PI/2, false), 0),
        LATENCY,
        w,
        0,
        Timer.getFPGATimestamp() * Constants.SECONDS_TO_MS);

    SmartDashboard.putNumber("KFx", this.kalmanFilter.getPose().x);
    SmartDashboard.putNumber("KFy", this.kalmanFilter.getPose().y);

    SmartDashboard.putNumber("Ox", this.odometry.x);
    SmartDashboard.putNumber("Oy", this.odometry.y);

    this.odometry.setPose(
        new Pose2d(this.kalmanFilter.getPose().vec(), this.gyro.getAngle() * Math.PI / 180));
    this.kalmanFilter.setHeading(this.gyro.getAngle() * Math.PI / 180);
  }
}
