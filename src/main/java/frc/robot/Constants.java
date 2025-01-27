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

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.Enums.Side;
import frc.robot.util.Enums.Source;
import java.util.HashMap;
import java.util.Map;

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
    public static final Map<Source, Pose2d> sourcePosesBlue = new HashMap<>();
    public static final Map<Side, Pose2d> sidePosesBlue = new HashMap<>();

    public static final Map<Source, Pose2d> sourcePosesRed = new HashMap<>();
    public static final Map<Side, Pose2d> sidePosesRed = new HashMap<>();

    static {
      /* BLUE */

      sourcePosesBlue.put(Source.SOURCE_1, new Pose2d(1.15, 1.10, Rotation2d.fromDegrees(53.5)));
      sourcePosesBlue.put(Source.SOURCE_2, new Pose2d(1.15, 7, Rotation2d.fromDegrees(-53.5)));
      sidePosesBlue.put(Side.SIDE_1, new Pose2d(3.15, 4, Rotation2d.fromDegrees(0)));
      sidePosesBlue.put(Side.SIDE_2, new Pose2d(3.80, 5.15, Rotation2d.fromDegrees(-60)));
      sidePosesBlue.put(Side.SIDE_3, new Pose2d(5.15, 5.20, Rotation2d.fromDegrees(-120)));
      sidePosesBlue.put(Side.SIDE_4, new Pose2d(5.85, 4, Rotation2d.fromDegrees(180)));
      sidePosesBlue.put(Side.SIDE_5, new Pose2d(5.15, 2.85, Rotation2d.fromDegrees(120)));
      sidePosesBlue.put(Side.SIDE_6, new Pose2d(3.80, 2.85, Rotation2d.fromDegrees(60)));

      /* RED */
      // TODO: add red side poses and source poses
      sidePosesRed.put(Side.SIDE_1, new Pose2d(14.45, 4, Rotation2d.fromDegrees(0)));
    }
  }
}
