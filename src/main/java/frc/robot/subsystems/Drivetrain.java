// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.mineinjava.quail.RobotMovement;
import com.mineinjava.quail.util.geometry.Vec2d;
import com.studica.frc.AHRS;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.math.Constants;
import java.util.ArrayList;

public class Drivetrain extends SubsystemBase {

  private AHRS gyro;

  private QuailSwerveDrive quailSwerveDrive;

  private ArrayList<QuailSwerveModule> modules;

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

  public void XLock() {
    quailSwerveDrive.XLock();
  }

  public void stop() {
    quailSwerveDrive.stop();
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
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }
}
