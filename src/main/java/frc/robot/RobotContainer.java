// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Autos;
import frc.robot.commands.Drive;
import frc.robot.math.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ElevatorArm;
import frc.robot.subsystems.LEDSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

  // The robot's subsystems and commands are defined here...
  public final Drivetrain drivetrain = new Drivetrain(gyro);
  public final Climber climber = new Climber();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController =
      new CommandXboxController(Constants.DRIVER_PORT);

  private final CommandXboxController operatorController =
      new CommandXboxController(Constants.OPERATOR_PORT);

  public final ElevatorArm elevator =
      new ElevatorArm(
          Constants.ELEVATOR_IDS[0],
          Constants.ELEVATOR_IDS[1],
          Constants.ARM_MOTOR_ID,
          Constants.HOPPER_SWITCH_ID,
          Constants.ARM_SWITCH_ID);

  public Autos autos = new Autos(drivetrain, elevator);

  public final LEDSubsystem led = new LEDSubsystem(elevator);

  // private final CommandXboxController operatorController =
  //     new CommandXboxController(Constants.OPERATOR_PORT);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    SmartDashboard.putData(elevator);
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //    .onTrue(new ExampleCommand(m_exampleSubsystem));

    Drive drive = new Drive(drivetrain, driverController);

    drivetrain.setDefaultCommand(drive);
    driverController.back().onTrue(drivetrain.resetGyroCommand());
    driverController.back().onTrue(led.blinkLEDs());

    driverController.x().onTrue(drivetrain.xLockCommand());
    
    //driverController.a().whileTrue(drivetrain.autoAlign());
    driverController.y().whileTrue(drivetrain.disableMotorBreakingCommand());
    driverController.y().whileFalse(drivetrain.enableMotorBreakingCommand());

    driverController.leftBumper().onTrue(this.climber.setSpeed(Constants.CLIMBER_SPEED));
    driverController.leftBumper().onFalse(this.climber.setSpeed(0));

    driverController.rightBumper().onTrue(this.climber.setSpeed(-Constants.CLIMBER_SPEED));
    driverController.rightBumper().onFalse(this.climber.setSpeed(0));

    // Debugging only
    // driverController.a().whileTrue(new DriveForward(drivetrain));

    // Elevator & Arm Controls
    operatorController.y().onTrue(this.elevator.L4Sequence());
    operatorController.x().onTrue(this.elevator.L1Sequence());
    operatorController.a().onTrue(this.elevator.L2Sequence());
    operatorController.b().onTrue(this.elevator.L3Sequence());

    operatorController.povDown().onTrue(this.elevator.pickupSequence());
    operatorController.povUp().onTrue(this.elevator.intakePosition());
    operatorController.povRight().onTrue(this.elevator.setArmPositionSCORE_HIGH());
    operatorController.povLeft().onTrue(this.elevator.setArmPositionSCORE_LOW());

    operatorController.rightBumper().onTrue(this.elevator.setElevatorPositionStow());
    operatorController.leftBumper().onTrue(this.elevator.setArmPositionStow());

    // Y Up is negative for some reason
    operatorController
        .axisLessThan(1, -Constants.JOYSTICK_DEAD_ZONE)
        .whileTrue(this.elevator.elevatorUp());
    operatorController
        .axisGreaterThan(1, Constants.JOYSTICK_DEAD_ZONE)
        .whileTrue(this.elevator.elevatorDown());
    operatorController
        .axisGreaterThan(1, -Constants.JOYSTICK_DEAD_ZONE)
        .onTrue(this.elevator.elevatorStop());
    operatorController
        .axisLessThan(1, Constants.JOYSTICK_DEAD_ZONE)
        .onTrue(this.elevator.elevatorStop());

    operatorController
        .axisGreaterThan(4, Constants.JOYSTICK_DEAD_ZONE)
        .whileTrue(this.elevator.armUp());
    operatorController
        .axisLessThan(4, -Constants.JOYSTICK_DEAD_ZONE)
        .whileTrue(this.elevator.armDown());
    operatorController
        .axisGreaterThan(4, -Constants.JOYSTICK_DEAD_ZONE)
        .onTrue(this.elevator.armStop());
    operatorController
        .axisLessThan(4, Constants.JOYSTICK_DEAD_ZONE)
        .onTrue(this.elevator.armStop());

    /*
    operatorController.povUp().whileTrue(this.elevator.elevatorUp());
    operatorController.povDown().whileTrue(this.elevator.elevatorDown());
    operatorController.povLeft().whileTrue(this.elevator.armDown());
    operatorController.povRight().whileTrue(this.elevator.armUp());

    operatorController.povUp().onFalse(this.elevator.elevatorStop());
    operatorController.povDown().onFalse(this.elevator.elevatorStop());
    operatorController.povLeft().onFalse(this.elevator.armStop());
    operatorController.povRight().onFalse(this.elevator.armStop());
    */

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  // An example command will be run in autonomous
  // return Autos.exampleAuto(m_exampleSubsystem);
  // }
}
