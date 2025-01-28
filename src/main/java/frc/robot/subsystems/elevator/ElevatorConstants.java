package frc.robot.subsystems.elevator;

import com.pathplanner.lib.config.PIDConstants;

public final class ElevatorConstants {
  public static final int masterMotorPort = 5; // TODO: Check the port numbers
  public static final int slaveMotorPort = 6;
  public static final double kGearRatio = 10.0;

  public static final PIDConstants kSimPIDConstants = new PIDConstants(0.015, 0, 0);

  public static final double kTolerance = 3; // cm

  public static final double gearRadius = 0.025; // meters TODO: Check this value
  public static final double maxHeight = 80;
  public static final double weight = 15; // kg

  public static final double HomeCurrent = 40; // TODO: Check this value
  public static final double MaxCurrent = 80; // TODO: Check this value

  public static final double kP = 0;
  public static final double kI = 0;
  public static final double kD = 0;
  public static final double kS = 0;
  public static final double kV = 0;
  public static final double kA = 0;
  public static final double kG = 0;
  // Motion Magic
  public static final double kMMCruiseVelocity = 0;
  public static final double kMMAcceleration = 0;
  public static final double kMMJerk = 0;
}
