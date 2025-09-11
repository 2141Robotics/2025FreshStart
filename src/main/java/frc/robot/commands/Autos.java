// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.mineinjava.quail.util.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.math.Constants;
import frc.robot.math.FieldMeasurements;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ElevatorArm;
import java.util.ArrayList;

public class Autos {
  Drivetrain drivetrain;
  ElevatorArm elevator;

  /**
   * Example static factory for an autonomous command. public static Command exampleAuto(Drivetrain
   * subsystem) { return Commands.sequence(subsystem.exampleMethodCommand(), new
   * ExampleCommand(subsystem)); }
   */
  public Autos(Drivetrain dt, ElevatorArm ele) {
    this.drivetrain = dt;
    this.elevator = ele;
  }

  public Command DoNothing() {
    return new WaitCommand(0);
  }

  public Command TestTaxi() {
    ArrayList<Pose2d> point = new ArrayList<Pose2d>();
    point.add(new Pose2d(3.11, -135.95 + Constants.robotWidth/2 + 10.25, 0));
    return new SequentialCommandGroup(
        this.drivetrain.resetGyroCommand(),
        this.elevator.setArmPositionStow(),
        new RunPath(this.drivetrain, point));
  }

  public Command TaxiCenter() {
    ArrayList<Pose2d> points = new ArrayList<Pose2d>();
    points.add(new Pose2d(6.8, -135.95 + Constants.robotWidth/2 + 40, 0));
    return new SequentialCommandGroup(
        this.drivetrain.resetGyroCommand(),
        this.elevator.setArmPositionStow(),
        new RunPath(this.drivetrain, points));
  }

  public Command TaxiLeft() {
    ArrayList<Pose2d> points = new ArrayList<Pose2d>();
    points.add(new Pose2d(-120, -110, 0));
    return new SequentialCommandGroup(
        this.drivetrain.resetGyroCommand(),
        this.elevator.setArmPositionStow(),
        new RunPath(this.drivetrain, points));
  }

  public Command TaxiRight() {
    ArrayList<Pose2d> points = new ArrayList<Pose2d>();
    points.add(new Pose2d(120, -110, 0));
    return new SequentialCommandGroup(
        this.drivetrain.resetGyroCommand(),
        this.elevator.setArmPositionStow(),
        new RunPath(this.drivetrain, points));
  }

  public Command REDScoreL4() {
    ArrayList<Pose2d> points1 = new ArrayList<Pose2d>();
    ArrayList<Pose2d> backupPoints1 = new ArrayList<Pose2d>();
    // points1.add(new Pose2d(3, -111,0));
    points1.add(new Pose2d(3.11, -135.95 + Constants.robotWidth/2 + 10.25, 0));
    backupPoints1.add(new Pose2d(3.11, -135.95 + Constants.robotWidth/2 + 10.25 + 10, 0));
    return new SequentialCommandGroup(
        this.drivetrain.resetGyroCommand(),
        this.elevator.L4Sequence(),
        new WaitCommand(0.5),
        new RunPath(drivetrain, points1));//,
        /*
        this.elevator.setArmPositionOUT(),
        new WaitCommand(0.5),
        new RunPath(drivetrain, backupPoints1),
        new WaitCommand(0.5),
        this.elevator.setElevatorPositionStow(),
        this.elevator.setArmPositionStow());
        */
  }

  public Command BLUEScoreL4() {
    ArrayList<Pose2d> score1 = new ArrayList<Pose2d>();
    // points1.add(new Pose2d(3, -111,0));
    score1.add(
        new Pose2d(
            (FieldMeasurements.G_H_POLE_WIDTH / 2),
            -FieldMeasurements.CENTER_TO_G_H_TROUGH + (Constants.robotWidth / 2) + 10.25d,
            0));
    ArrayList<Pose2d> back1 = new ArrayList<Pose2d>();
    back1.add(
        new Pose2d(
            (FieldMeasurements.G_H_POLE_WIDTH / 2),
            -FieldMeasurements.CENTER_TO_G_H_TROUGH + (Constants.robotWidth / 2) + 10.25d + 10d,
            0));
    return new SequentialCommandGroup(
        this.drivetrain.resetGyroCommand(),
        this.elevator.L4Sequence(),
        new WaitCommand(2),
        new RunPath(drivetrain, score1),
        this.elevator.setArmPositionOUT(),
        new WaitCommand(2),
        new RunPath(drivetrain, back1),
        new WaitCommand(2),
        this.elevator.setElevatorPositionStow(),
        this.elevator.setArmPositionStow());
  }

  public Command BLUEScore2L4() {
    ArrayList<Pose2d> score1 = new ArrayList<Pose2d>();
    ArrayList<Pose2d> back1 = new ArrayList<Pose2d>();
    ArrayList<Pose2d> intake1 = new ArrayList<Pose2d>();
    ArrayList<Pose2d> intermediate1 = new ArrayList<Pose2d>();
    ArrayList<Pose2d> score2 = new ArrayList<Pose2d>();
    ArrayList<Pose2d> back2 = new ArrayList<Pose2d>();
    ArrayList<Pose2d> end = new ArrayList<Pose2d>();
    score1.add(new Pose2d(38.6918, -153.823, Constants.PI / 3));
    back1.add(new Pose2d(47.352, -148.823, Constants.PI / 3));
    intermediate1.add(new Pose2d(71.352, -148.823, Constants.PI / 2));
    // 126 degrees converted to radians
    intake1.add(new Pose2d(126.816, -281.029, 0.7d * Constants.PI));
    score2.add(new Pose2d(38.7308, -183.584, 2 * Constants.PI / 3));
    back2.add(new Pose2d(47.391, -188.584, 2 * Constants.PI / 3));
    end.add(new Pose2d(47.391, -188.584, Constants.PI));
    return new SequentialCommandGroup(
        this.drivetrain.resetGyroCommand(),
        this.elevator.L4Sequence(),
        new WaitCommand(0.5),
        new RunPath(drivetrain, score1),
        this.elevator.setArmPositionOUT(),
        new WaitCommand(0.5),
        new RunPath(drivetrain, back1),
        new RunPath(drivetrain, intermediate1),
        this.elevator.intakePosition(),
        new RunPath(drivetrain, intake1),
        new WaitCommand(3),
        this.elevator.pickupSequence(),
        new WaitCommand(1),
        this.elevator.L4Sequence(),
        new RunPath(drivetrain, score2),
        this.elevator.setArmPositionOUT(),
        new WaitCommand(0.5),
        new RunPath(drivetrain, back2),
        this.elevator.setElevatorPositionStow(),
        this.elevator.setArmPositionStow());
  }
}
