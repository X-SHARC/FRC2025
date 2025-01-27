package frc.robot.subsystems.elevator;

public final class ElevatorConstants {
  public static final int masterMotorPort = 5;
  public static final int slaveMotorPort = 6;
  public static final double kGearRatio = 10.0; // Changed from 0.10 to 10.0 for 1:10 ratio
  public static final double kP = 0.015;
  public static final double kI = 0;
  public static final double kD = 0;
  public static final double gearRadius = 0.025; // m
  public static final double gearCircumference = 2 * gearRadius * Math.PI;
  public static final double kTolerance = 3; // m
  public static final double maxHeight = 80;
  public static final double weight = 4;
  public static final double maxVelocity = 10;
}
