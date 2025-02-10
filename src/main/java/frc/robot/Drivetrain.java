
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    modules.add(new QuailSwerveModule(new Vec2d(Constants.CENTER_TO_SWERVE_DIST, Constants.CENTER_TO_SWERVE_DIST), 1, 2, 10, 0));
    modules.add(new QuailSwerveModule(new Vec2d(Constants.CENTER_TO_SWERVE_DIST, -Constants.CENTER_TO_SWERVE_DIST), 3, 4, 11, 0));
    modules.add(new QuailSwerveModule(new Vec2d(-Constants.CENTER_TO_SWERVE_DIST, -Constants.CENTER_TO_SWERVE_DIST), 5, 6, 12, 0));
    modules.add(new QuailSwerveModule(new Vec2d(-Constants.CENTER_TO_SWERVE_DIST, Constants.CENTER_TO_SWERVE_DIST), 7, 8, 13, 0));

    this.quailSwerveDrive = new QuailSwerveDrive(gyro, modules);
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

  public void init(){
    quailSwerveDrive.initModules();

  }

  public void drive(RobotMovement robotMovement){
    if(canDrive()){
      quailSwerveDrive.drive(robotMovement);
    }
  }

  public void stop(){
    quailSwerveDrive.stop();
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
