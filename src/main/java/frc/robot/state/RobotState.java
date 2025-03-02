package frc.robot.state;

import frc.robot.util.Enums.GameObject;
import frc.robot.util.Enums.Height;
import frc.robot.util.Enums.OperationMode;
import frc.robot.util.Enums.Position;
import org.littletonrobotics.junction.AutoLogOutput;

public class RobotState {

  private static RobotState instance;

  private RobotState() {
  }

  private static OperationMode currentMode = OperationMode.HUMAN;
  private static GameObject currentGameObject = GameObject.NONE;
  private static Position selectedPosition = Position.LEFT;
  private static Height selectedElevatorHeight = Height.ZERO;
  private static double elevatorHeight = 0.0;

  /* SETTERS */

  public static void setMode(OperationMode mode) {
    currentMode = mode;
  }

  public static void setGameObject(GameObject object) {
    currentGameObject = object;
  }

  public static void setSelectedPosition(Position position) {
    selectedPosition = position;
  }

  public static void setSelectedElevatorHeight(Height height) {
    selectedElevatorHeight = height;
  }

  public static void setElevatorHeight(double height) {
    elevatorHeight = height;
  }

  /* GETTERS */

  @AutoLogOutput(key = "RobotState/OperationMode")
  public static OperationMode getMode() {
    return currentMode;
  }

  @AutoLogOutput(key = "RobotState/GameObject")
  public static GameObject getGameObject() {
    return currentGameObject;
  }

  @AutoLogOutput(key = "RobotState/SelectedBranchPosition")
  public static Position getSelectedPosition() {
    return selectedPosition;
  }

  @AutoLogOutput(key = "RobotState/ElevatorHeight")
  public static double getElevatorHeight() {
    return elevatorHeight;
  }

  @AutoLogOutput(key = "RobotState/ElevatorHeightSpeedMultiplier")
  public static double getSpeedMultiplier() {
    return (1 - elevatorHeight);
  }

  @AutoLogOutput(key = "RobotState/SelectedElevatorHeight")
  public static Height getSelectedElevatorHeight() {
    return selectedElevatorHeight;
  }

  public static boolean isAuto() {
    return currentMode == OperationMode.AUTO;
  }

  public static boolean hasObject() {
    return currentGameObject != GameObject.NONE;
  }

  /* RESET */

  public static void reset() {
    currentMode = OperationMode.HUMAN;
    currentGameObject = GameObject.NONE;
    selectedPosition = Position.LEFT;
    elevatorHeight = 0.0;
    selectedElevatorHeight = Height.ZERO;
  }

  public static RobotState getInstance() {
    if (instance == null) {
      instance = new RobotState();
    }
    return instance;
  }
}
