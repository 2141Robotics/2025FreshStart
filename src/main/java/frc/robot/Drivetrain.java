
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radian;

import java.util.ArrayList;

import com.mineinjava.quail.RobotMovement;
import com.mineinjava.quail.util.geometry.Vec2d;
import com.studica.frc.AHRS;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.QuailSwerveDrive;
import frc.robot.subsystems.QuailSwerveModule;
import frc.robot.math.Constants;

public class Drivetrain extends SubsystemBase {

  private AHRS gyro;

  private QuailSwerveDrive quailSwerveDrive;

  /** Creates a new ExampleSubsystem. */
  public Drivetrain(AHRS gyro) {

    this.gyro = gyro;
    
    ArrayList<QuailSwerveModule> modules = new ArrayList<>();

    //TODO Set cancoder offsets
    modules.add(new QuailSwerveModule(new Vec2d(Constants.CENTER_TO_SWERVE_DIST, Constants.CENTER_TO_SWERVE_DIST), 1, 2, Constants.CANCODER_IDS[0], Constants.CANCODER_OFFSETS[0]));
    modules.add(new QuailSwerveModule(new Vec2d(Constants.CENTER_TO_SWERVE_DIST, -Constants.CENTER_TO_SWERVE_DIST), 3, 4, Constants.CANCODER_IDS[1], Constants.CANCODER_OFFSETS[1]));
    modules.add(new QuailSwerveModule(new Vec2d(-Constants.CENTER_TO_SWERVE_DIST, -Constants.CENTER_TO_SWERVE_DIST), 5, 6, Constants.CANCODER_IDS[2], Constants.CANCODER_OFFSETS[2]));
    modules.add(new QuailSwerveModule(new Vec2d(-Constants.CENTER_TO_SWERVE_DIST, Constants.CENTER_TO_SWERVE_DIST), 7, 8, Constants.CANCODER_IDS[3], Constants.CANCODER_OFFSETS[3]));

    this.quailSwerveDrive = new QuailSwerveDrive(modules);
  }

  /**
	 * Checks if the robot is resetting or if the gyro is callibrating.
	 * 
	 * @return Whether or not the robot can drive
	 */
	public boolean canDrive()
	{
		return !this.gyro.isCalibrating();
	}

  public void reset(){
    quailSwerveDrive.reset();
    resetGyro();
    System.out.println("RESET DRIVETRAIN");
  }

  public void init(){
    quailSwerveDrive.initModules();
    resetGyro();
  }

  public void drive(RobotMovement robotMovement){
    if(canDrive()){
      quailSwerveDrive.drive(robotMovement, this.getGyroAngle());
    }
  }

  public void XLock(){
    quailSwerveDrive.XLock();
  }

  public void stop(){
    quailSwerveDrive.stop();
  }


	public Angle getGyroAngle(){
    double gyroAngle = this.gyro.getAngle();
    Angle angle = Angle.ofBaseUnits(gyroAngle, Degrees);
    System.out.println("Gyro Angle: " + gyroAngle);
    return angle;
  }

  
	/**
	 * Reset the gyro to 0°.
	 */
	public void resetGyro()
	{
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
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
