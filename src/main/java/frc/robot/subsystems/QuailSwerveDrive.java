package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.StaticBrake;
import com.mineinjava.quail.RobotMovement;
import com.mineinjava.quail.SwerveDrive;
import com.mineinjava.quail.util.geometry.Vec2d;
import frc.robot.math.Constants;
import java.util.ArrayList;
import java.util.List;

public class QuailSwerveDrive extends SwerveDrive<QuailSwerveModule> {

  /** The gryoscope used for rotaiton measurements. */
  /** A list of all of the swerve modules on the drivetrain. */
  private final List<QuailSwerveModule> modules;

  public Orchestra orchestra;

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
    orchestra = new Orchestra();
  }

  public void initModules() {
    this.modules.forEach(m -> m.init());
    this.setBrake(new StaticBrake());
  }

  public void initMusic() {

    for (QuailSwerveModule module : this.modules) {
      orchestra.addInstrument(module.getSteerMotor());
      orchestra.addInstrument(module.getDriveMotor());
    }
    orchestra.loadMusic(Constants.SONG_NAME);
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
    // Commented out because it doesnt work for some reason,
    // manually implemented instead
    // XLockModules();
    for (int i = 0; i < Constants.X_LOCK_POSITIONS.length; i++) {
      modules.get(i).reset();
      //modules.get(i).setRawAngle(Constants.X_LOCK_POSITIONS[i].in(Radian));
    }
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

  public void setMotorSound(int frequency) {
    for (QuailSwerveModule module : this.modules) {
      module.setMotorSound(frequency);
    }
  }

  public ArrayList<Vec2d> getModuleSpeeds() {
    ArrayList<Vec2d> vectors = new ArrayList<Vec2d>();
    for (QuailSwerveModule module : this.swerveModules) {
      vectors.add(((QuailSwerveModule) module).getCurrentMovement());
    }
    return vectors;
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
