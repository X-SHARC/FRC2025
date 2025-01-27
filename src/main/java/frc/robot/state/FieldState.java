// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.state;

import frc.robot.util.Enums.Height;
import frc.robot.util.Enums.PoleMaxHeight;
import frc.robot.util.Enums.Position;
import frc.robot.util.Enums.Side;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

/**
 * Manages the state of the field during gameplay, tracking which poles are filled or empty at
 * different heights and positions. This class follows the Singleton pattern to ensure only one
 * instance of the field state exists.
 *
 * <p>The state is stored in a three-level map structure:
 *
 * <ul>
 *   <li>Level 1: Side of the field
 *   <li>Level 2: Position (LEFT/RIGHT)
 *   <li>Level 3: Height level and its occupancy state
 * </ul>
 */
public class FieldState {

  private static FieldState fieldState = new FieldState();

  private static final Map<Side, Map<Position, Map<Height, Boolean>>> fieldStates;

  static {
    fieldStates = new HashMap<>();
    initialize();
  }

  /**
   * Private constructor to prevent direct instantiation. Use {@link #getInstance()} to access the
   * singleton instance.
   */
  private FieldState() {}

  /**
   * Marks a specific position on the field as filled.
   *
   * @param side the side of the field
   * @param position LEFT or RIGHT position
   * @param height the height level to mark as filled
   */
  public static void setFilled(Side side, Position position, Height height) {
    Map<Position, Map<Height, Boolean>> positions = fieldStates.get(side);
    if (positions != null) {
      Map<Height, Boolean> heights = positions.get(position);
      if (heights != null) {
        heights.put(height, true);
        Logger.recordOutput("FieldState/Sides/" + side + "/" + position + "/" + height, true);
      }
    }
  }

  /**
   * Marks a specific position on the field as empty.
   *
   * @param side the side of the field
   * @param position LEFT or RIGHT position
   * @param height the height level to mark as empty
   */
  public static void setEmpty(Side side, Position position, Height height) {
    Map<Position, Map<Height, Boolean>> positions = fieldStates.get(side);
    if (positions != null) {
      Map<Height, Boolean> heights = positions.get(position);
      if (heights != null) {
        heights.put(height, false);
        Logger.recordOutput("FieldState/Sides/" + side + "/" + position + "/" + height, false);
      }
    }
  }

  /**
   * Checks if a specific position on the field is empty.
   *
   * @param side the side of the field
   * @param position LEFT or RIGHT position
   * @param height the height level to check
   * @return true if the position is empty or unknown, false if it's filled
   */
  public static boolean isEmpty(Side side, Position position, Height height) {
    Map<Position, Map<Height, Boolean>> positions = fieldStates.get(side);
    if (positions != null) {
      Map<Height, Boolean> heights = positions.get(position);
      if (heights != null) {
        Boolean state = heights.get(height);
        return state == null || !state;
      }
    }
    return true;
  }

  /**
   * Determines the maximum available height for a specific pole position. Scans from top to bottom
   * to find the first empty position.
   *
   * @param side the side of the field
   * @param position LEFT or RIGHT position
   * @return the maximum available height for the specified pole
   */
  public static PoleMaxHeight getMaxAvailablePoleHeight(Side side, Position position) {
    Map<Position, Map<Height, Boolean>> positions = fieldStates.get(side);
    if (positions != null) {
      Map<Height, Boolean> heights = positions.get(position);
      if (heights != null) {
        // Start from highest level and work down
        for (int i = Height.values().length - 1; i >= 0; i--) {
          Height height = Height.fromValue(i);
          Boolean isFilled = heights.get(height);
          if (isFilled == null || !isFilled) {
            Logger.recordOutput("FieldState/Sides/" + side + "/" + position + "/Max", height);
            // If we find an empty spot, return the corresponding PoleMaxHeight
            return i == 0 ? PoleMaxHeight.FULL : PoleMaxHeight.fromValue(i);
          }
        }
      }
    }
    return PoleMaxHeight.FULL;
  }

  /**
   * Gets the maximum available height for a given side of the field, comparing both LEFT and RIGHT
   * positions.
   *
   * @param side the side of the field to check
   * @return the higher of the two available heights between LEFT and RIGHT positions
   */
  public static PoleMaxHeight getMaxAvailableHeight(Side side) {
    PoleMaxHeight left = getMaxAvailablePoleHeight(side, Position.LEFT);
    PoleMaxHeight right = getMaxAvailablePoleHeight(side, Position.RIGHT);

    // Return the higher of the two available heights
    return left.getValue() > right.getValue() ? left : right;
  }

  /**
   * Gets the singleton instance of FieldState.
   *
   * @return the singleton instance of FieldState
   */
  public static FieldState getInstance() {
    return fieldState;
  }

  /**
   * Resets the entire field state to empty. This method clears all existing state and reinitializes
   * the field with all positions marked as empty.
   */
  public static void initialize() {
    fieldStates.clear();
    for (Side side : Side.values()) {
      Map<Position, Map<Height, Boolean>> positions = new HashMap<>();
      for (Position position : Position.values()) {
        if (position == Position.MIDDLE) {
          continue; // Skip CENTER position
        }
        Map<Height, Boolean> heights = new HashMap<>();
        for (Height height : Height.values()) {
          heights.put(height, false); // Set all heights as empty (false)
          Logger.recordOutput("FieldState/Sides/" + side + "/" + position + "/" + height, false);
        }
        positions.put(position, heights);
        Logger.recordOutput("FieldState/Sides/" + side + "/" + position + "/Max", Height.L4);
      }
      fieldStates.put(side, positions);
    }
  }
}
