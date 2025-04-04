// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.mineinjava.quail.RobotMovement;
import com.mineinjava.quail.util.geometry.Vec2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.math.Constants;
import frc.robot.subsystems.Drivetrain;

/** An example command that uses an example subsystem. */
public class Drive extends Command {

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
  public void initialize() {
    System.out.println("Drive command started");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Gets Joystick values
    double leftX = driverController.getLeftX();
    double leftY = -driverController.getLeftY(); // Y UP is negative
    double rightX = driverController.getRightX();
    double rightY = -driverController.getRightY();

    Vec2d leftStickVector = new Vec2d(leftX, leftY);
    Vec2d rightStickVector = new Vec2d(rightX, rightY);

    double scaledSpeed =
        leftStickVector.getLength() * 0.1 + driverController.getRightTriggerAxis() * 0.9;

    if (leftStickVector.getLength() < Constants.JOYSTICK_DEAD_ZONE) {
      leftStickVector = new Vec2d(0, 0);
    }
    if (rightStickVector.getLength() < Constants.JOYSTICK_DEAD_ZONE) {
      rightStickVector = new Vec2d(0, 0);
    }

    Vec2d driveVector = leftStickVector.normalize().scale(scaledSpeed);

    if ((Math.abs(rightStickVector.x) < Constants.JOYSTICK_DEAD_ZONE)
        && (leftStickVector.getLength() < Constants.JOYSTICK_DEAD_ZONE)) {
      drivetrain.stop();
    } else {

      RobotMovement movement =
          new RobotMovement(
              -rightStickVector.x / Constants.ROTATION_SPEED_INVERSE_SCALE, driveVector);
      drivetrain.drive(movement);

      SmartDashboard.putNumber("driveVecX", driveVector.x);
      SmartDashboard.putNumber("driveVecY", driveVector.y);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Drive Command Ended. Interrupted: " + interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
