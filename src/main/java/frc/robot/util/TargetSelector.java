package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;
import frc.robot.state.RobotState;
import frc.robot.util.Enums.Position;
import org.littletonrobotics.junction.Logger;

public class TargetSelector {

  private static TargetSelector instance = new TargetSelector();
  private static Pose2d currentBotPose = new Pose2d();
  private static Pose2d currentClosestSide = new Pose2d();
  private static Pose2d currentClosestSource = new Pose2d();

  private TargetSelector() {
  }

  private static Pose2d getBranchPose(Position branchPosition, Pose2d sidePose) {
    Pose2d flippedPose = sidePose;
    double sideAngle = sidePose.getRotation().getRadians();

    return switch (branchPosition) {
      case LEFT -> new Pose2d(
          sidePose.getX() - Math.sin(sideAngle) * 0.175,
          sidePose.getY() + Math.cos(sideAngle) * 0.175,
          sidePose.getRotation());
      case RIGHT -> new Pose2d(
          sidePose.getX() + Math.sin(sideAngle) * 0.175,
          sidePose.getY() - Math.cos(sideAngle) * 0.175,
          sidePose.getRotation());
      case MIDDLE -> flippedPose;
      default -> throw new IllegalArgumentException("Unknown branch position: " + branchPosition);
    };
  }

  private static void updateNearestSide() {
    currentClosestSide = AllianceFlipUtil.apply(currentBotPose.nearest(Constants.FieldConstants.SIDEPOSE_LIST));
  }

  public static Pose2d getNearestSide() {
    return currentClosestSide;
  }

  public static Pose2d getNearestBranch(Position branchPosition) {
    return getBranchPose(branchPosition, currentClosestSide);
  }

  private static void updateNearestSource() {
    currentClosestSource = currentBotPose.nearest(Constants.FieldConstants.SOURCEPOSE_LIST);
  }

  public static Pose2d getNearestSourcePose() {
    return currentClosestSource;
  }

  public static void update(Pose2d pose) {
    currentBotPose = AllianceFlipUtil.apply(pose);
    updateNearestSide();
    updateNearestSource();
    Logger.recordOutput("TargetSelector/Current Closest Side", currentClosestSide);
    Logger.recordOutput(
        "TargetSelector/Current Closest Branch",
        getNearestBranch(RobotState.getSelectedPosition()));
  }

  public static TargetSelector getInstance() {
    if (instance == null) {
      instance = new TargetSelector();
    }
    return instance;
  }
};
