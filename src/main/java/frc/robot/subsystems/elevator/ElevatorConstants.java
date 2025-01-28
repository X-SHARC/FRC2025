package frc.robot.subsystems.elevator;

import com.pathplanner.lib.config.PIDConstants;

public final class ElevatorConstants {
  public static final int masterMotorPort = 5; // TODO: Check the port numbers
  public static final int slaveMotorPort = 6;
  public static final double kGearRatio = 10.0;

  public static final PIDConstants kSimPIDConstants = new PIDConstants(0.015, 0, 0);
  public static final PIDConstants kPIDConstants =
      new PIDConstants(0, 0, 0); // TODO: Tune the values

  public static final double kTolerance = 3; // cm

  public static final double gearRadius = 0.025; // meters TODO: Check this value
  public static final double maxHeight = 80;
  public static final double weight = 4;

  public static final double HomeCurrent = 40; // TODO: Check this value
  public static final double MaxCurrent = 80; // TODO: Check this value
}
