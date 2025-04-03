package frc.robot.commands;

import com.mineinjava.quail.RobotMovement;
import com.mineinjava.quail.pathing.ConstraintsPair;
import com.mineinjava.quail.pathing.Path;
import com.mineinjava.quail.pathing.PathFollower;
import com.mineinjava.quail.util.MiniPID;
import com.mineinjava.quail.util.geometry.Pose2d;
import com.mineinjava.quail.util.geometry.Vec2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.math.Constants;
import frc.robot.subsystems.Drivetrain;

import static edu.wpi.first.units.Units.Newton;

import java.util.ArrayList;

public class RunPath extends Command {
  protected Drivetrain drivetrain;
  protected PathFollower pathfollower;
  private Path path;
  private ArrayList<Pose2d> points;
  private MiniPID pidController;

  public RunPath(Drivetrain drivetrain, ArrayList<Pose2d> points) {
    this.points = points;
    this.drivetrain = drivetrain;
    // TODO: Move to constants + tune
    this.pidController = new MiniPID(6, 0.0, 1.2);
    this.pidController.setDeadband(Constants.ANGLE_PRECISION);
    this.pidController.setF(0);

    addRequirements(drivetrain);

    System.out.println("Constructed runPath Command w/ PF: " + pathfollower);
  }

  @Override
  public void initialize() {
    this.path = new Path(this.points);


    // TODO: Put units on these
    ConstraintsPair translationPair = new ConstraintsPair(40, 1000);
    ConstraintsPair rotationPair = new ConstraintsPair(0.5, .5);

    this.pathfollower =
        new PathFollower(
            this.drivetrain.getOdometry(),
            this.path,
            translationPair,
            rotationPair,
            this.pidController,
            1,
            1,
            1,
            1,
            15);

    this.path.currentPointIndex = 0;
    System.out.println("Initialized RunPath Command...");
  }

  @Override
  public void execute() {
    SmartDashboard.putNumber("RunPath Index", this.path.currentPointIndex);
    RobotMovement nextMovement = pathfollower.calculateNextDriveMovement();
    Vec2d newTranslation =
        (new Vec2d(nextMovement.translation.x / 200, nextMovement.translation.y / 200));
    double rotation = nextMovement.rotation / 100; // TODO: De magic this number!!

    //System.out.println("p: " + this.pathfollower.getPath().currentPointIndex);
    //System.out.println("p: " + newTranslation.toString());

    drivetrain.drive(new RobotMovement(-rotation, newTranslation));
  }

  @Override
  public boolean isFinished() {
    return (this.path.isFinished());
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Run path completed - was interrupted: " + interrupted);
    // drivetrain.stop();
    super.end(interrupted);
  }
}
