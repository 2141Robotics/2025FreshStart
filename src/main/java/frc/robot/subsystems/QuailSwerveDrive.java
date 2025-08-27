package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.StaticBrake;
import com.mineinjava.quail.RobotMovement;
import com.mineinjava.quail.SwerveDrive;
import frc.robot.math.Constants;
import java.util.List;

public class QuailSwerveDrive extends SwerveDrive<QuailSwerveModule> {

  /** The gryoscope used for rotaiton measurements. */
  /** A list of all of the swerve modules on the drivetrain. */
  private final List<QuailSwerveModule> modules;

  /**
   * @param minSpeed minimum movement speed (0 to 1)
   * @param maxSpeed maximum movement speed (0 to 1)
   * @param maxRotation maximum rotational speed (0 to 1)
   * @param gyroscope the swerve drive's gyroscope
   * @param QuailSwerveModules the swerve drive's wheel modules
   */
  public QuailSwerveDrive(List<QuailSwerveModule> modules) {
    super(modules);
    this.modules = modules;
  }

  public void initModules() {
    this.modules.forEach(m -> m.init());
    this.setBrake(new StaticBrake());
  }

  public void reset() {
    this.modules.forEach(m -> m.reset());
  }

  public void drive(RobotMovement robotMovement, double gyroAngleDeg) {
    move(robotMovement, Math.toRadians(gyroAngleDeg) + Constants.GYRO_OFFSET.in(Radians));
  }

  public List<QuailSwerveModule> getModules() {
    return this.modules;
  }

  public void XLock() {
    XLockModules();
  }

  public void stop() {
    for (QuailSwerveModule module : this.modules) {
      module.setRawSpeed(0);
    }
    setBrake(new StaticBrake());
  }

  public void setBrake(ControlRequest brake) {
    for (QuailSwerveModule module : this.modules) {
      module.setBrake(brake);
    }
  }

  @Override
  public String toString() {
    // The class will be represented as "SwerveDrive[Module1 = {}, Module2 = {},
    // ...]"
    StringBuilder builder = new StringBuilder("SwerveDrive[");

    for (int i = 0; i < this.modules.size(); i++) {
      builder.append("Module" + i + " = {" + this.modules.get(i) + "}, ");
    }

    builder.delete(builder.length() - 2, builder.length());
    builder.append("]");
    return builder.toString();
  }
}
