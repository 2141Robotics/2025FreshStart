// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import org.ejml.equation.Sequence;

import com.mineinjava.quail.util.geometry.Pose2d;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ElevatorArm;

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

  public Command noop() {
    return new WaitCommand(0);
  }

  public Command TaxiCenter(){
    ArrayList<Pose2d> points = new ArrayList<Pose2d>();
    points.add(new Pose2d(0,-100,0));
    return new SequentialCommandGroup(
      this.elevator.setArmPositionStow(),
      new RunPath(this.drivetrain, points));
  }

  public Command TaxiLeft(){
    ArrayList<Pose2d> points = new ArrayList<Pose2d>();
    points.add(new Pose2d(-120,-100,0));
    return new SequentialCommandGroup(
      this.elevator.setArmPositionStow(),
      new RunPath(this.drivetrain, points));  }

  public Command TaxiRight(){
    ArrayList<Pose2d> points = new ArrayList<Pose2d>();
    points.add(new Pose2d(120,-100,0));
    return new SequentialCommandGroup(
      this.elevator.setArmPositionStow(),
      new RunPath(this.drivetrain, points));  }

  public Command testAngles() {
    ArrayList<Pose2d> points = new ArrayList<Pose2d>();
    points.add(new Pose2d(0,30,Math.PI/2));
    points.add(new Pose2d(0,0,0));
    return new SequentialCommandGroup(
      this.elevator.setArmPositionStow(),
      new RunPath(this.drivetrain, points));  }

  public Command ScoreL4() {
    ArrayList<Pose2d> points1 = new ArrayList<Pose2d>();
   // points1.add(new Pose2d(3, -111,0));
    points1.add(new Pose2d(-9,-134,0));
    ArrayList<Pose2d> points2 = new ArrayList<Pose2d>();
    points2.add(new Pose2d(0, -100,0));
    return new SequentialCommandGroup(
      this.elevator.L4Sequence(),
      new WaitCommand(0.5),
      new RunPath(drivetrain, points1),
      this.elevator.setArmPositionOUT(),
      new WaitCommand(0.5),
      new RunPath(drivetrain, points2),
      new WaitCommand(0.5),
      this.elevator.L4Sequence(),
      new WaitCommand(0.5),
      new RunPath(drivetrain, points1),
      this.elevator.setArmPositionOUT(),
      new WaitCommand(0.5),
      new RunPath(drivetrain, points2),
      new WaitCommand(0.5),
      this.elevator.setElevatorPositionStow(),
      this.elevator.setArmPositionStow()
    );
  }
}
