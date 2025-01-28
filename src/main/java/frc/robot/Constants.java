// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import java.util.Map;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.Enums.Height;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always
 * "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics
 * sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public final class GeneratorConstants {
    public static final double centerToPoleDist = 1.0;
    public static final PathConstraints constraints = new PathConstraints(
        4.98, 4.5, Units.degreesToRadians(360), Units.degreesToRadians(720), 12);
  }

  public static final class FieldConstants {

    public static final double fieldLength = Units.inchesToMeters(690.876);
    public static final double fieldWidth = Units.inchesToMeters(317);

    public static final double reefOffset = 0.1645;

    public static final Pose2d[] sourcePoses = new Pose2d[] {
        new Pose2d(1.15, 1.10, Rotation2d.fromDegrees(53.5)),
        new Pose2d(1.15, 7, Rotation2d.fromDegrees(-53.5))
    };
    public static final Pose2d[] sidePoses = new Pose2d[] {
        new Pose2d(3.15, 4, Rotation2d.fromDegrees(0)),
        new Pose2d(3.80, 5.15, Rotation2d.fromDegrees(-60)),
        new Pose2d(5.15, 5.20, Rotation2d.fromDegrees(-120)),
        new Pose2d(5.85, 4, Rotation2d.fromDegrees(180)),
        new Pose2d(5.15, 2.85, Rotation2d.fromDegrees(120)),
        new Pose2d(3.80, 2.85, Rotation2d.fromDegrees(60))
    };

    public static final Map<Height, Number> heightMap = Map.of(
        Height.ZERO, 0.0,
        Height.L1, 0,
        Height.L2, 25.0,
        Height.L3, 40.0,
        Height.L4, 80.0);
  }
}
