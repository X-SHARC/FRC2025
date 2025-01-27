package frc.robot.state;

import frc.robot.util.Enums.GameObject;
import frc.robot.util.Enums.Height;
import frc.robot.util.Enums.OperationMode;
import frc.robot.util.Enums.SideOverride;

public class RobotState {

  private static RobotState instance;

  private RobotState() {}

  private static OperationMode currentMode = OperationMode.HUMAN;
  private static Height currentElevatorHeight = Height.ZERO;
  private static GameObject currentGameObject = GameObject.NONE;
  private static SideOverride currentSideOverride = SideOverride.NONE;

  /* SETTERS */

  public static void setMode(OperationMode mode) {
    currentMode = mode;
  }

  public static void setElevatorHeight(Height height) {
    currentElevatorHeight = height;
  }

  public static void setGameObject(GameObject object) {
    currentGameObject = object;
  }

  public static void setSideOverride(SideOverride side) {
    currentSideOverride = side;
  }

  /* GETTERS */

  public static OperationMode getMode() {
    return currentMode;
  }

  public static Height getElevatorHeight() {
    return currentElevatorHeight;
  }

  public static GameObject getGameObject() {
    return currentGameObject;
  }

  public static SideOverride getSideOverride() {
    return currentSideOverride;
  }

  public static boolean isAuto() {
    return currentMode == OperationMode.AUTO;
  }

  public static boolean hasObject() {
    return currentGameObject != GameObject.NONE;
  }

  public static void cycleSideOverride() {
    currentSideOverride = SideOverride.cycle(currentSideOverride);
  }

  /* RESET */

  public static void reset() {
    currentMode = OperationMode.HUMAN;
    currentElevatorHeight = Height.ZERO;
    currentGameObject = GameObject.NONE;
    currentSideOverride = SideOverride.NONE;
  }

  public static RobotState getInstance() {
    if (instance == null) {
      instance = new RobotState();
    }
    return instance;
  }
}
