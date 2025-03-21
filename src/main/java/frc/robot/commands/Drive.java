// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.mineinjava.quail.RobotMovement;
import com.mineinjava.quail.util.geometry.AccelerationLimitedDouble;
import com.mineinjava.quail.util.geometry.AccelerationLimitedVector;
import com.mineinjava.quail.util.geometry.Vec2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.math.Constants;
import frc.robot.subsystems.Drivetrain;

/** An example command that uses an example subsystem. */
public class Drive extends Command {

  public AccelerationLimitedVector a_leftStickVector =
      new AccelerationLimitedVector(Constants.MOVEMENT_ACCELERATION_LIMIT);

  public AccelerationLimitedVector a_rightStickVector =
      new AccelerationLimitedVector(Constants.ROTATION_ACCELERATION_LIMIT);

  public AccelerationLimitedVector a_driveVector =
      new AccelerationLimitedVector(Constants.DRIVE_ACCELERATION_LIMIT);

  public AccelerationLimitedDouble a_rtrigger =
      new AccelerationLimitedDouble(Constants.SPEED_ACCELERATION_LIMIT);

  private Drivetrain drivetrain;

  private CommandXboxController driverController;

  /** Creates a new ExampleCommand. */
  public Drive(Drivetrain drivetrain, CommandXboxController driverController) {

    this.drivetrain = drivetrain;
    this.driverController = driverController;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Gets Joystick values
    double leftX = driverController.getLeftX();
    double leftY = -driverController.getLeftY(); // Y UP is negative
    double rightX = driverController.getRightX();
    double rightY = -driverController.getRightY();

    double rightTrigger = driverController.getRightTriggerAxis();
    double leftTrigger = driverController.getLeftTriggerAxis();

    // Makes joystick values into Vectors
    Vec2d leftStickVector = new Vec2d(leftX, leftY);
    Vec2d rightStickVector = new Vec2d(rightX, rightY);

    // Updates accelerationLimitedVector(s) and acceleration limits current vectors
    Vec2d leftStick = a_leftStickVector.update(leftStickVector);
    Vec2d rightStick = a_rightStickVector.update(rightStickVector);
    // double a_rtriggerValue = a_rtrigger.update(rightTrigger);

    double scaledSpeed = Constants.BASE_SPEED;

    // Left Trigger (Slow speed) has prioroty
    if (rightTrigger > 0.08 && leftTrigger > 0.08) {
      scaledSpeed = Constants.BASE_SPEED - (Constants.BASE_SPEED * leftTrigger);
    } else if (rightTrigger > 0) {
      scaledSpeed = Constants.BASE_SPEED + ((1 - Constants.BASE_SPEED) * rightTrigger);
    } else if (leftTrigger > 0) {
      scaledSpeed = Constants.BASE_SPEED - (Constants.BASE_SPEED * leftTrigger);
    }

    if (scaledSpeed < Constants.MINIMUM_SPEED) {
      scaledSpeed = Constants.MINIMUM_SPEED;
    }

    if (leftStick.getLength() < Constants.JOYSTICK_DEAD_ZONE) {
      leftStickVector = new Vec2d(0, 0);
    }
    if (rightStick.getLength() < Constants.JOYSTICK_DEAD_ZONE) {
      rightStickVector = new Vec2d(0, 0);
    }

    Vec2d driveVector = leftStickVector.normalize().scale(scaledSpeed);
    Vec2d newDriveVector = a_driveVector.update(driveVector);

    if ((Math.abs(rightStick.x) < Constants.JOYSTICK_DEAD_ZONE)
        && (leftStick.getLength() < Constants.JOYSTICK_DEAD_ZONE)) {
      drivetrain.stop();
    } else {

      RobotMovement movement =
          new RobotMovement(
              rightStickVector.x / Constants.ROTATION_SPEED_INVERSE_SCALE, newDriveVector);
      drivetrain.drive(movement);

      SmartDashboard.putNumber("driveVecX", newDriveVector.x);
      SmartDashboard.putNumber("driveVecY", newDriveVector.y);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
