// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.math.Constants;

public class Climber extends SubsystemBase {

  private final TalonFX climberMotor;

  /** Creates a new ExampleSubsystem. */
  public Climber() {

    climberMotor = new TalonFX(Constants.CLIMBER_MOTOR_ID);
    climberMotor.setNeutralMode(NeutralModeValue.Brake);
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
    super.periodic();
  }

  public void setRawSpeed(double speed) {
    this.climberMotor.setControl(new DutyCycleOut(speed));
  }

  public Command setSpeed(double speed) {
    return this.runOnce(
        () -> {
          this.setRawSpeed(speed);
        });
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }
}
