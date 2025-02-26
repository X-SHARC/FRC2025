package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.GeneratorConstants;
import frc.robot.util.Enums.Position;
import frc.robot.util.Enums.Side;
import frc.robot.util.Enums.Source;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class SharcGen {

  public SharcGen() {
    initializeMaps();
  }

  private Map<Source, Pose2d> sourcePoses = new HashMap<>();
  private Map<Side, Pose2d> sidePoses = new HashMap<>();

  private Side selectSide() {
    double maxWeight = 0;
    Side selectedSide = Side.SIDE_1;
    Pose2d currentPose = AutoBuilder.getCurrentPose();

    for (Map.Entry<Side, Pose2d> entry : sidePoses.entrySet()) {
      Pose2d sidePose = AllianceFlipUtil.apply(entry.getValue());
      double distance = currentPose.getTranslation().getDistance(sidePose.getTranslation());
      double weight = distance == 0 ? 1 : 1 / distance;
      if (weight > maxWeight) {
        maxWeight = weight;
        selectedSide = entry.getKey();
      }
    }
    return selectedSide;
  }

  private double getDistanceToSource(Pose2d currentPose, Source source) {
    return currentPose
        .getTranslation()
        .getDistance(AllianceFlipUtil.apply(sourcePoses.get(source)).getTranslation());
  }

  private Source selectSource() {
    Pose2d currentPose = AutoBuilder.getCurrentPose();
    double distanceToSource1 = getDistanceToSource(currentPose, Source.SOURCE_1);
    double distanceToSource2 = getDistanceToSource(currentPose, Source.SOURCE_2);
    return distanceToSource1 <= distanceToSource2 ? Source.SOURCE_1 : Source.SOURCE_2;
  }

  public Pose2d getPolePose(Side side, Position position) {
    Pose2d sidePose = AllianceFlipUtil.apply(sidePoses.get(side)); // Apply AllianceFlip
    if (position == Position.MIDDLE) {
      return sidePose;
    }
    double multiplier = position == Position.LEFT ? -.175 : .175;
    double sideAngle = sidePose.getRotation().getRadians();
    return new Pose2d(
        sidePose.getX() - Math.sin(sideAngle) * multiplier,
        sidePose.getY() + Math.cos(sideAngle) * multiplier,
        sidePose.getRotation());
  }

  // public Command generateSidePath(Position pos) {
  // Side selectedSide = selectSide();
  // String pathName = selectedSide.toString() + "_" + pos.toString();
  // PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

  // // Pose2d poseToGo = getPolePose(selectedSide, pos);
  // // return AutoBuilder.pathfindToPose(poseToGo,
  // GeneratorConstants.constraints,
  // // 0);
  // return AutoBuilder.pathfindThenFollowPath(path,
  // GeneratorConstants.constraints);
  // }

  public Command generateSidePath(Position pos) {
    Side selectedSide = selectSide();
    Pose2d sidePoseI = AllianceFlipUtil.apply(sidePoses.get(selectedSide));
    Pose2d sidePose = getPolePose(selectedSide, pos);

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(sidePoseI, sidePose);
    PathPlannerPath path =
        new PathPlannerPath(
            waypoints,
            GeneratorConstants.constraints,
            null,
            new GoalEndState(0, sidePose.getRotation()));

    path.preventFlipping = true; // TODO: Check if this is necessary
    return AutoBuilder.followPath(path);

    // String pathName = selectedSide.toString() + "_" + pos.toString();

    // try {
    // PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
    // return AutoBuilder.pathfindThenFollowPath(path,
    // GeneratorConstants.constraints);
    // } catch (IOException e) {
    // System.err.println("Failed to load path file: " + pathName);
    // e.printStackTrace();
    // } catch (ParseException e) {
    // System.err.println("Failed to parse path file: " + pathName);
    // e.printStackTrace();
    // }

    // return Commands.none();
  }

  public Command generateSourcePath() {
    Source selectedSource = selectSource();
    return AutoBuilder.pathfindToPose(
        AllianceFlipUtil.apply(sourcePoses.get(selectedSource)), GeneratorConstants.constraints, 0);
  }

  private void initializeMaps() {

    System.out.println(FieldConstants.sourcePoses);

    for (int i = 0; i < FieldConstants.sourcePoses.length; i++) {

      sourcePoses.put(Source.fromValue(i + 1), FieldConstants.sourcePoses[i]);
    }

    System.out.println(sourcePoses);

    for (int i = 0; i < FieldConstants.sidePoses.length; i++) {

      sidePoses.put(Side.fromValue(i + 1), FieldConstants.sidePoses[i]);
    }
  }
}
