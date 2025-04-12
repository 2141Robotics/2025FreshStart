package frc.robot.math;

import edu.wpi.first.math.geometry.Translation2d;

public class FieldMeasurements {

  /**
   * Note: This file uses this orientation and nomenclature G, I, and J are points on the trough
   * just infront of the L4 posts that allow all other posts' positions to be derived from them
   *
   * <p>DRIVERS -------------
   *
   *         __________
   *        /  B    A  \ 
   *       /C          L\ 
   *      /D     Reef   K\
   *      \E            J/ 
   *       \F          I/ 
   *        \__H____G__/
   *
   * <p>Center of field (0,0)
   *
   * <p>^ | -y | x+ <--------> x- | | +y v
   */

  // Official value: 47.947"
  public static final double CENTER_TO_BARGE_ZONE_EDGE = 47.947d;

  // Official value: 65.49"
  public static final double REEF_WIDTH = 65.49d;

  // Official value: 88"
  public static final double BARGE_ZONE_EDGE_TO_G_H_TROUGH = 88d;

  // Official value: 12.94"
  public static final double G_H_POLE_WIDTH = 12.94d;

  // Center of field to edge of the barge zone
  public static final Translation2d CENTER_TO_BARGE_EDGE =
      new Translation2d(0, -(CENTER_TO_BARGE_ZONE_EDGE));

  // Edge of the barge zone to the front lip of the G_H Trough
  public static final Translation2d BARGE_EDGE_TO_G_H_TROUGH =
      new Translation2d(0, -(BARGE_ZONE_EDGE_TO_G_H_TROUGH));

  // Distance from center of trough to the center of the reef
  public static final Translation2d G_H_TROUGH_TO_CENTER_REEF =
      new Translation2d(0, -(REEF_WIDTH / 2));

  public static final Translation2d CENTER_TO_CENTER_REEF =
      CENTER_TO_BARGE_EDGE.plus(BARGE_EDGE_TO_G_H_TROUGH.plus(G_H_TROUGH_TO_CENTER_REEF));

  // How far to the side post A is from the center of the trough
  public static final Translation2d G_H_TROUGH_TO_G = new Translation2d(G_H_POLE_WIDTH / 2, 0);

  public static final Translation2d G =
      new Translation2d(-G_H_TROUGH_TO_G.getX(), -G_H_TROUGH_TO_CENTER_REEF.getY())
          .plus(CENTER_TO_CENTER_REEF);
  public static final Translation2d H =
      new Translation2d(G_H_TROUGH_TO_G.getX(), -G_H_TROUGH_TO_CENTER_REEF.getY())
          .plus(CENTER_TO_CENTER_REEF);
  public static final Translation2d A =
      new Translation2d(-G_H_TROUGH_TO_G.getX(), G_H_TROUGH_TO_CENTER_REEF.getY())
          .plus(CENTER_TO_CENTER_REEF);
  public static final Translation2d B =
      new Translation2d(G_H_TROUGH_TO_G.getX(), G_H_TROUGH_TO_CENTER_REEF.getY())
          .plus(CENTER_TO_CENTER_REEF);

  public static final Translation2d I_J_TROUGH_TO_CENTER_REEF =
      new Translation2d(
          (G_H_TROUGH_TO_CENTER_REEF.getY()) * Math.sin(Constants.PI / 3),
          (G_H_TROUGH_TO_CENTER_REEF.getY()) * Math.cos(Constants.PI / 3));

  public static final Translation2d I =
      new Translation2d(
              (G_H_TROUGH_TO_G.getX()) * Math.sin(Constants.PI / 3),
              (G_H_TROUGH_TO_G.getX()) * Math.cos(Constants.PI / 3))
          .plus(I_J_TROUGH_TO_CENTER_REEF.plus(CENTER_TO_CENTER_REEF));
  public static final Translation2d J =
      new Translation2d(
              -(G_H_TROUGH_TO_G.getX()) * Math.sin(Constants.PI / 3),
              -(G_H_TROUGH_TO_G.getX()) * Math.cos(Constants.PI / 3))
          .plus(I_J_TROUGH_TO_CENTER_REEF.plus(CENTER_TO_CENTER_REEF));

  public static final Translation2d L =
      new Translation2d(
              (G_H_TROUGH_TO_G.getX()) * Math.sin(Constants.PI / 3),
              -(G_H_TROUGH_TO_G.getX()) * Math.cos(Constants.PI / 3))
          .plus(
              new Translation2d(I_J_TROUGH_TO_CENTER_REEF.getX(), -I_J_TROUGH_TO_CENTER_REEF.getY())
                  .plus(CENTER_TO_CENTER_REEF));
  public static final Translation2d K =
      new Translation2d(
              -(G_H_TROUGH_TO_G.getX()) * Math.sin(Constants.PI / 3),
              (G_H_TROUGH_TO_G.getX()) * Math.cos(Constants.PI / 3))
          .plus(
              new Translation2d(I_J_TROUGH_TO_CENTER_REEF.getX(), -I_J_TROUGH_TO_CENTER_REEF.getY())
                  .plus(CENTER_TO_CENTER_REEF));

  public static final Translation2d F =
      new Translation2d(
              -(G_H_TROUGH_TO_G.getX()) * Math.sin(Constants.PI / 3),
              (G_H_TROUGH_TO_G.getX()) * Math.cos(Constants.PI / 3))
          .plus(
              new Translation2d(-I_J_TROUGH_TO_CENTER_REEF.getX(), I_J_TROUGH_TO_CENTER_REEF.getY())
                  .plus(CENTER_TO_CENTER_REEF));
  public static final Translation2d E =
      new Translation2d(
              (G_H_TROUGH_TO_G.getX()) * Math.sin(Constants.PI / 3),
              -(G_H_TROUGH_TO_G.getX()) * Math.cos(Constants.PI / 3))
          .plus(
              new Translation2d(-I_J_TROUGH_TO_CENTER_REEF.getX(), I_J_TROUGH_TO_CENTER_REEF.getY())
                  .plus(CENTER_TO_CENTER_REEF));

  public static final Translation2d C =
      new Translation2d(
              -(G_H_TROUGH_TO_G.getX()) * Math.sin(Constants.PI / 3),
              -(G_H_TROUGH_TO_G.getX()) * Math.cos(Constants.PI / 3))
          .plus(
              new Translation2d(
                      -I_J_TROUGH_TO_CENTER_REEF.getX(), -I_J_TROUGH_TO_CENTER_REEF.getY())
                  .plus(CENTER_TO_CENTER_REEF));
  public static final Translation2d D =
      new Translation2d(
              (G_H_TROUGH_TO_G.getX()) * Math.sin(Constants.PI / 3),
              (G_H_TROUGH_TO_G.getX()) * Math.cos(Constants.PI / 3))
          .plus(
              new Translation2d(
                      -I_J_TROUGH_TO_CENTER_REEF.getX(), -I_J_TROUGH_TO_CENTER_REEF.getY())
                  .plus(CENTER_TO_CENTER_REEF));
}
