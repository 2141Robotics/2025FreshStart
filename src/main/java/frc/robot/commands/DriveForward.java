// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.mineinjava.quail.RobotMovement;
import com.mineinjava.quail.util.geometry.Vec2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

/** An example command that uses an example subsystem. */
public class DriveForward extends Command {

  private Drivetrain drivetrain;

  /** Creates a new ExampleCommand. */
  public DriveForward(Drivetrain drivetrain) {

    this.drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Starting drive forward");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotMovement movement = new RobotMovement(0, new Vec2d(0, .05));
    drivetrain.drive(movement);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    System.out.println("Finished drive forward");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
