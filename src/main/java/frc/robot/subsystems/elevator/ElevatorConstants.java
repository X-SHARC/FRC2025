package frc.robot.subsystems.elevator;

import com.pathplanner.lib.config.PIDConstants;

public final class ElevatorConstants {
  public static final int masterMotorPort = 13;
  public static final int slaveMotorPort = 14;
  public static final double kGearRatio = 7; // 10 for sim

  public static final PIDConstants kSimPIDConstants = new PIDConstants(0.015, 0, 0);

  public static final double kTolerance = 3; // cm

  public static final double drumRadius = 0.02432; // meters
  public static final double maxHeight = 80;
  public static final double weight = 15; // kg

  public static final double HomeCurrent = 40;
  public static final double MaxCurrent = 100;

  public static final double kP = 0.5;
  public static final double kI = 0;
  public static final double kD = 0.1;
  public static final double kS = 0.45;
  public static final double kV = 0.146;
  public static final double kA = 0;
  public static final double kG = 0.35;
  // Motion Magic
  public static final double kMMCruiseVelocity = 80;
  public static final double kMMAcceleration = 160;
  public static final double kMMJerk = 1600;
}
