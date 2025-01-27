package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.GeneratorConstants;
import frc.robot.commands.ElevatorSet;
import frc.robot.state.FieldState;
import frc.robot.state.RobotState;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.util.Enums.Height;
import frc.robot.util.Enums.OperationMode;
import frc.robot.util.Enums.PoleMaxHeight;
import frc.robot.util.Enums.Position;
import frc.robot.util.Enums.Side;
import frc.robot.util.Enums.Source;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class SharcGen {
  private Map<Source, Pose2d> sourcePoses;
  private Map<Side, Pose2d> sidePoses;

  public SharcGen() {
    initializeMaps();
  }

  private void initializeMaps() {
    boolean isRedAlliance =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    sourcePoses =
        isRedAlliance
            ? Constants.FieldConstants.sourcePosesRed
            : Constants.FieldConstants.sourcePosesBlue;
    sidePoses =
        isRedAlliance
            ? Constants.FieldConstants.sidePosesRed
            : Constants.FieldConstants.sidePosesBlue;
  }

  public Pose2d getPolePose(Side side, Position position) {
    double multiplier = position == Position.LEFT ? -.175 : .175;
    Pose2d sidePose = sidePoses.get(side);
    double sideAngle = sidePose.getRotation().getRadians();
    // return sidePose;
    return new Pose2d(
        sidePose.getX() - Math.sin(sideAngle) * multiplier,
        sidePose.getY() + Math.cos(sideAngle) * multiplier,
        sidePose.getRotation());
  }

  public Command generateSourcePath() {
    Source selectedSource = selectSource();
    return AutoBuilder.pathfindToPose(
        sourcePoses.get(selectedSource), GeneratorConstants.constraints, 0);
  }

  public Command generateSourcePath(Source source) {
    return AutoBuilder.pathfindToPose(sourcePoses.get(source), GeneratorConstants.constraints, 0);
  }

  private Source selectSource() {
    // TODO: Implement override logic
    Source selectedSource = selectNearestSource();
    return selectedSource;
  }

  private Source selectNearestSource() {
    Pose2d currentPose = AutoBuilder.getCurrentPose();
    double distanceToSource1 = getDistanceToSource(currentPose, Source.SOURCE_1);
    double distanceToSource2 = getDistanceToSource(currentPose, Source.SOURCE_2);
    return distanceToSource1 <= distanceToSource2 ? Source.SOURCE_1 : Source.SOURCE_2;
  }

  private double getDistanceToSource(Pose2d currentPose, Source source) {
    return currentPose.getTranslation().getDistance(sourcePoses.get(source).getTranslation());
  }

  public Command generateSidePath() {
    Side selectedSide = selectSide();
    Position selectedPosition = selectPosition(selectedSide);
    Pose2d poseToGo = getPolePose(selectedSide, selectedPosition);
    return AutoBuilder.pathfindToPose(poseToGo, GeneratorConstants.constraints, 0);
  }

  public Command generateSidePath(Side side, Position position) {
    return AutoBuilder.pathfindToPose(
        getPolePose(side, position), GeneratorConstants.constraints, 0);
  }

  private Side selectSide() {
    // TODO: Implement override logic
    Side selectedSide = selectOptimalSide();
    return selectedSide;
  }

  private Side selectOptimalSide() {
    double maxWeight = 0;
    Side selectedSide = Side.SIDE_1;
    Pose2d currentPose = AutoBuilder.getCurrentPose();

    for (Map.Entry<Side, Pose2d> entry : sidePoses.entrySet()) {
      double weight = getWeight(entry.getKey(), currentPose);
      if (weight > maxWeight) {
        maxWeight = weight;
        selectedSide = entry.getKey();
      }
    }
    return selectedSide;
  }

  private double getWeight(Side side, Pose2d currentPose) {
    Pose2d sidePose = sidePoses.get(side);
    double distance = currentPose.getTranslation().getDistance(sidePose.getTranslation());
    double dW = distance == 0 ? 1 : 1 / distance;

    PoleMaxHeight poleMaxHeight = FieldState.getMaxAvailableHeight(side);
    double hW = poleMaxHeight.getValue();

    double weight = dW * 8 + hW;
    return weight;
  }

  private Position selectPosition(Side side) {
    PoleMaxHeight leftHeight = FieldState.getMaxAvailablePoleHeight(side, Position.LEFT);
    PoleMaxHeight rightHeight = FieldState.getMaxAvailablePoleHeight(side, Position.RIGHT);

    return leftHeight.getValue() > rightHeight.getValue() ? Position.LEFT : Position.RIGHT;
  }

  public void updateGoalPositionAndHeight(Side selectedSide, Position selectedPosition) {

    PoleMaxHeight selectedHeight =
        FieldState.getMaxAvailablePoleHeight(selectedSide, selectedPosition);

    if (selectedHeight != PoleMaxHeight.FULL) {
      FieldState.setFilled(selectedSide, selectedPosition, selectedHeight.toHeight());
    }

    Logger.recordOutput("SharcGen/ActiveSide/Override", RobotState.getSideOverride().toString());
    Logger.recordOutput("SharcGen/ActiveSide/Side", selectedSide.toString());
    Logger.recordOutput("SharcGen/ActiveSide/Position", selectedPosition.toString());
    Logger.recordOutput("SharcGen/ActiveSide/Height", selectedHeight.getValue());
  }

  public Command generateCoralPath(Side side, Position position) {
    if (RobotState.hasObject()) {
      return generateSidePath(side, position);
    } else {
      return generateSourcePath();
    }
  }

  public SequentialCommandGroup generateHalfCoralCycle(Elevator elevator) {
    boolean hasObject = RobotState.hasObject();
    Side selectedSide = selectSide();

    Position selectedPosition = selectPosition(selectedSide);

    PoleMaxHeight selectedHeight =
        FieldState.getMaxAvailablePoleHeight(selectedSide, selectedPosition);

    if (selectedHeight == PoleMaxHeight.FULL) {
      RobotState.setMode(OperationMode.HUMAN);
      return new SequentialCommandGroup();
    }

    Height elevatorSetHeight = hasObject ? selectedHeight.toHeight() : Height.ZERO;

    SequentialCommandGroup cycle =
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                generateCoralPath(selectedSide, selectedPosition), new ElevatorSet(elevator, 0)),
            new ElevatorSet(elevator, elevatorSetHeight.getValue() * 15));

    if (hasObject) {
      cycle.setName("Coral Place");
      cycle.addCommands(
          new InstantCommand(() -> updateGoalPositionAndHeight(selectedSide, selectedPosition)));
    } else {
      cycle.setName("Coral Pickup");
    }

    cycle.initialize();
    return cycle;
  }
}
