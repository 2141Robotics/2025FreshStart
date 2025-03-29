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
import java.util.ArrayList;

public class RunPath extends Command {
  protected Drivetrain drivetrain;
  protected PathFollower pathfollower;
  private Path path;
  private MiniPID pidController;

  public RunPath(Drivetrain drivetrain, ArrayList<Pose2d> points) {
    this.path = new Path(points);
    this.drivetrain = drivetrain;
    // TODO: Move to constants + tune
    this.pidController = new MiniPID(3, 0.0, 1.2);
    this.pidController.setDeadband(Constants.ANGLE_PRECISION);
    this.pidController.setF(0);

    addRequirements(drivetrain);

    System.out.println("Constructed runPath Command w/ PF: " + pathfollower);
  }

  @Override
  public void initialize() {

    // TODO: Put units on these
    ConstraintsPair translationPair = new ConstraintsPair(80, 1000);
    ConstraintsPair rotationPair = new ConstraintsPair(0.5, .5);

    this.pathfollower =
        new PathFollower(
            this.drivetrain.getOdometry(),
            this.path,
            translationPair,
            rotationPair,
            this.pidController,
            2,
            0.1,
            2,
            1,
            20);

    this.path.currentPointIndex = 0;
    System.out.println("Initialized RunPath Command...");
  }

  @Override
  public void execute() {
    SmartDashboard.putNumber("RunPath INdex", this.path.currentPointIndex);
    RobotMovement nextMovement = pathfollower.calculateNextDriveMovement();
    Vec2d newTranslation =
        (new Vec2d(nextMovement.translation.x / 200, nextMovement.translation.y / 200));
    double rotation = nextMovement.rotation / 100; // TODO: De magic this number!!

    SmartDashboard.putString("next movement", newTranslation.toString());

    drivetrain.drive(new RobotMovement(-rotation, newTranslation));
  }

  @Override
  public boolean isFinished() {
    return this.pathfollower.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Run path completed - was interrupted: " + interrupted);
    // drivetrain.stop();
    super.end(interrupted);
  }
}
