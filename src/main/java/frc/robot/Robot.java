// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.math.Constants;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final SendableChooser<Command> ledChooser = new SendableChooser<>();

  private final RobotContainer m_robotContainer;

  private Command lastCommand;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    ledChooser.setDefaultOption(
        "Off", this.m_robotContainer.leds.setPatternCommand(Constants.PATTERN_OFF));
    ledChooser.addOption(
        "Red", this.m_robotContainer.leds.setPatternCommand(Constants.PATTERN_RED));
    ledChooser.addOption(
        "Orange", this.m_robotContainer.leds.setPatternCommand(Constants.PATTERN_ORANGE));
    ledChooser.addOption(
        "Yellow", this.m_robotContainer.leds.setPatternCommand(Constants.PATTERN_YELLOW));
    ledChooser.addOption(
        "Green", this.m_robotContainer.leds.setPatternCommand(Constants.PATTERN_GREEN));
    ledChooser.addOption(
        "DLS Green", this.m_robotContainer.leds.setPatternCommand(Constants.PATTERN_DLS_GREEN));
    ledChooser.addOption(
        "Blue", this.m_robotContainer.leds.setPatternCommand(Constants.PATTERN_BLUE));
    ledChooser.addOption(
        "Purple", this.m_robotContainer.leds.setPatternCommand(Constants.PATTERN_PURPLE));
    ledChooser.addOption("Up", this.m_robotContainer.leds.setPatternCommand(Constants.PATTERN_UP));
    ledChooser.addOption(
        "Down", this.m_robotContainer.leds.setPatternCommand(Constants.PATTERN_DOWN));
    ledChooser.addOption(
        "Moving", this.m_robotContainer.leds.setPatternCommand(Constants.PATTERN_SCROLL));
    ledChooser.addOption(
        "Police", this.m_robotContainer.leds.setPatternCommand(Constants.PATTERN_POLICE));
    ledChooser.addOption(
        "Dots", this.m_robotContainer.leds.setPatternCommand(Constants.PATTERN_DOTS));
    ledChooser.addOption(
        "Rainbow",
        this.m_robotContainer.leds.setPatternCommand(Constants.PATTERN_RAINBOW_SCROLLING));
    ledChooser.addOption(
        "Fire", this.m_robotContainer.leds.setPatternCommand(Constants.PATTERN_FIRE));
    ledChooser.addOption(
        "Particles", this.m_robotContainer.leds.setPatternCommand(Constants.PATTERN_PARTICLES));
    ledChooser.addOption(
        "Aurora", this.m_robotContainer.leds.setPatternCommand(Constants.PATTERN_AURORA));
    SmartDashboard.putData("LED Pattern", ledChooser);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.

    // Check for changes in the selected LED pattern
    Command selectedCommand = ledChooser.getSelected();
    if (selectedCommand != null && !selectedCommand.equals(lastCommand)) {
      selectedCommand.schedule();
      lastCommand = selectedCommand;
    }

    CommandScheduler.getInstance().run();
  }

  @Override
  public void robotInit() {
    m_robotContainer.drivetrain.init();
    RobotController.setBrownoutVoltage(5.5);
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    this.m_robotContainer.leds.setBreatheCommand(true).schedule();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    this.m_robotContainer.leds.setBreatheCommand(false).schedule();
  }

  @Override
  public void testInit() {
    m_robotContainer.drivetrain.quailSwerveDrive.orchestra.play();
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
