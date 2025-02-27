package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorArm;

public class ElevatorMovement extends Command {
    ElevatorArm m_elevator;
    double speed;

    public ElevatorMovement(ElevatorArm elevator, double speed) {
        m_elevator = elevator;
        addRequirements(m_elevator);
        this.speed = speed;
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Starting elevator movement @ speed: " + this.speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        this.m_elevator.setRawElevatorSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Finished elevator movement @ speed: " + this.speed);
    this.m_elevator.stopElevator();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

    
}