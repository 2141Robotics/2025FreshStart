package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import java.util.ArrayList;
import java.util.List;
import java.util.ResourceBundle.Control;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.StaticBrake;
import com.mineinjava.quail.RobotMovement;
import com.mineinjava.quail.SwerveDrive;
import com.mineinjava.quail.util.geometry.Vec2d;
import com.studica.frc.AHRS;

import edu.wpi.first.units.measure.Angle;
import frc.robot.math.Constants;

public class QuailSwerveDrive extends SwerveDrive<QuailSwerveModule>{

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
	public QuailSwerveDrive(List<QuailSwerveModule> modules)
	{
		super(modules);
		this.modules = modules;
		initModules();
	}

    public void initModules(){
        this.modules.forEach(m -> m.init());
        this.setBrake(new StaticBrake());
    }

	public void reset(){
        this.modules.forEach(m -> m.reset());

	}

	public void drive(RobotMovement robotMovement, double gyroAngleDeg){
		System.out.println("PASSED THRU ANGLE " + gyroAngleDeg);
		move(robotMovement, Math.toRadians(gyroAngleDeg) + Constants.GYRO_OFFSET.in(Radians));
	}

    
	public List<QuailSwerveModule> getModules()
	{
		return this.modules;
	}


	public void XLock(){
		XLockModules();
	}

	public void stop(){
		for(QuailSwerveModule module : this.modules) {
			module.setRawSpeed(0);
		}
		setBrake(new StaticBrake());
	}

	public void setBrake(ControlRequest brake){
		for(QuailSwerveModule module : this.modules) {
			module.setBrake(brake);
		}
	}

	/*public ArrayList<Vec2d> getModuleSpeeds() {
        ArrayList<Vec2d> vectors = new ArrayList<Vec2d>();
        for (QuailSwerveModule module : this.swerveModules) {
            vectors.add(((QuailSwerveModule) module).getCurrentMovement());
        }
        return vectors;
    }*/


    
	/**
	 * Average encoder distance of the drive modules
	 * @return distance in encoder ticks before gear ratio
	 */
    /*
	public double averageDist() {
		int totaldist = 0;
		for (int i = 0; i <4; i++) {
			totaldist += this.modules.get(i).drivingMotor.getPosition().refresh().getValue();
		}
		return totaldist/4;
	}

	public void resetDistance(){
		for (QuailSwerveModule module : this.modules) {
			module.drivingMotor.setPosition(0d);
		}
	}*/

	@Override
	public String toString()
	{
		// The class will be represented as "SwerveDrive[Module1 = {}, Module2 = {}, ...]"
		StringBuilder builder = new StringBuilder("SwerveDrive[");

		for(int i = 0; i < this.modules.size(); i++)
		{
			builder.append("Module" + i + " = {" + this.modules.get(i) + "}, ");
		}

		builder.delete(builder.length() - 2, builder.length());
		builder.append("]");
		return builder.toString();
	}

}

