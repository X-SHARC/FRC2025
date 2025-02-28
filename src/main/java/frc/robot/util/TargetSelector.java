package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.FieldConstants;
import frc.robot.util.Enums.Position;
import frc.robot.util.Enums.Side;
import frc.robot.util.Enums.Source;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class TargetSelector {

  public TargetSelector() {
    initializeMaps();
  }

  private Map<Source, Pose2d> sourcePoses = new HashMap<>();
  private Map<Side, Pose2d> sidePoses = new HashMap<>();
  Side closestSide = Side.SIDE_1;
  Pose2d closestPoseLeft = new Pose2d();
  Pose2d closestPoseRight = new Pose2d();
  Pose2d closestPoseMiddle = new Pose2d();

  public void update(Pose2d currentPose) {
    closestSide = selectSide(currentPose);
    closestPoseLeft = getPolePose(closestSide, Position.LEFT);
    closestPoseRight = getPolePose(closestSide, Position.RIGHT);
    closestPoseMiddle = getPolePose(closestSide, Position.MIDDLE);
    Logger.recordOutput("ClosestSide", closestSide.toString());
  }

  public Side selectSide(Pose2d currentPose) {
    double maxWeight = 0;
    Side selectedSide = Side.SIDE_1;

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

  public Pose2d getPolePose(Side side, Position position) {
    Pose2d sidePose = AllianceFlipUtil.apply(sidePoses.get(side)); // Apply AllianceFlip
    if (position == Position.MIDDLE) {
      return sidePose;
    }
    double multiplier = position == Position.LEFT ? .175 : -.175;
    double sideAngle = sidePose.getRotation().getRadians();
    return new Pose2d(
        sidePose.getX() - Math.sin(sideAngle) * multiplier,
        sidePose.getY() + Math.cos(sideAngle) * multiplier,
        sidePose.getRotation());
  }

  public Pose2d getClosestSidePose(Position position) {
    if (position == Position.LEFT) return closestPoseLeft;
    else if (position == Position.RIGHT) return closestPoseRight;
    else return closestPoseMiddle;
  }

  private double getDistanceToSource(Pose2d currentPose, Source source) {
    return currentPose
        .getTranslation()
        .getDistance(AllianceFlipUtil.apply(sourcePoses.get(source)).getTranslation());
  }

  public Source selectSource(Pose2d currentPose) {
    double distanceToSource1 = getDistanceToSource(currentPose, Source.SOURCE_1);
    double distanceToSource2 = getDistanceToSource(currentPose, Source.SOURCE_2);
    return distanceToSource1 <= distanceToSource2 ? Source.SOURCE_1 : Source.SOURCE_2;
  }

  public Pose2d getClosestSourcePose(Pose2d currentPose) {
    Source selectedSource = selectSource(currentPose);
    return sourcePoses.get(selectedSource);
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
