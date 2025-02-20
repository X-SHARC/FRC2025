package frc.robot.state;

import frc.robot.util.Enums.GameObject;
import frc.robot.util.Enums.OperationMode;
import frc.robot.util.Enums.Position;

public class RobotState {

  private static RobotState instance;

  private RobotState() {
  }

  private static OperationMode currentMode = OperationMode.HUMAN;
  private static GameObject currentGameObject = GameObject.NONE;
  private static Position selectedPosition = Position.LEFT;

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

  /* GETTERS */

  public static OperationMode getMode() {
    return currentMode;
  }

  public static GameObject getGameObject() {
    return currentGameObject;
  }

  public static Position getSelectedPosition() {
    return selectedPosition;
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
  }

  public static RobotState getInstance() {
    if (instance == null) {
      instance = new RobotState();
    }
    return instance;
  }
}
