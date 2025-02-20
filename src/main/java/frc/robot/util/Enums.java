// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.stream.Stream;

/**
 * A utility class that contains enums and helper methods for managing
 * robot-specific constants and
 * modes.
 */
public class Enums {

  /** Represents the operation mode of the robot. */
  public enum OperationMode {
    HUMAN,
    AUTO
  }

  /** Represents the game objects that the robot interacts with. */
  public enum GameObject {
    NONE,
    CORAL,
    ALGEA
  }

  /** Represents possible positions of a side. */
  public enum Position {
    LEFT,
    RIGHT,
    MIDDLE
  }

  /** Interface for enums with associated integer values. */
  public interface ValueEnum {
    /**
     * Gets the integer value associated with this enum constant.
     *
     * @return the integer value
     */
    int getValue();
  }

  /** Represents the sides of the Reef. */
  public enum Side implements ValueEnum {
    SIDE_1(1),
    SIDE_2(2),
    SIDE_3(3),
    SIDE_4(4),
    SIDE_5(5),
    SIDE_6(6);

    private final int value;

    Side(int value) {
      this.value = value;
    }

    @Override
    public int getValue() {
      return value;
    }

    /**
     * Creates a {@code Side} instance from an integer value.
     *
     * @param value the integer value
     * @return the corresponding {@code Side}
     * @throws IllegalArgumentException if no matching {@code Side} exists
     */
    public static Side fromValue(int value) {
      return EnumUtils.fromValue(Side.class, value);
    }
  }

  /** Represents sources in the field. */
  public enum Source implements ValueEnum {
    SOURCE_1(1),
    SOURCE_2(2);

    private final int value;

    Source(int value) {
      this.value = value;
    }

    @Override
    public int getValue() {
      return value;
    }

    /**
     * Creates a {@code Source} instance from an integer value.
     *
     * @param value the integer value
     * @return the corresponding {@code Source}
     * @throws IllegalArgumentException if no matching {@code Source} exists
     */
    public static Source fromValue(int value) {
      return EnumUtils.fromValue(Source.class, value);
    }
  }

  /** Represents height levels. */
  public enum Height implements ValueEnum {
    ZERO(0),
    L1(1),
    L2(2),
    L3(3),
    L4(4);

    private final int value;

    Height(int value) {
      this.value = value;
    }

    @Override
    public int getValue() {
      return value;
    }

    /**
     * Creates a {@code Height} instance from an integer value.
     *
     * @param value the integer value
     * @return the corresponding {@code Height}
     * @throws IllegalArgumentException if no matching {@code Height} exists
     */
    public static Height fromValue(int value) {
      return EnumUtils.fromValue(Height.class, value);
    }
  }

  /** Represents maximum height that is avalible on a pole of the Reef. */
  public enum PoleMaxHeight implements ValueEnum {
    FULL(-999),
    L1(1),
    L2(2),
    L3(3),
    L4(4);

    private final int value;

    PoleMaxHeight(int value) {
      this.value = value;
    }

    @Override
    public int getValue() {
      return value;
    }

    public Height toHeight() {
      return EnumUtils.fromValue(Height.class, this.value);
    }

    /**
     * Creates a {@code PoleMaxHeight} instance from an integer value.
     *
     * @param value the integer value
     * @return the corresponding {@code PoleMaxHeight}
     * @throws IllegalArgumentException if no matching {@code PoleMaxHeight} exists
     */
    public static PoleMaxHeight fromValue(int value) {
      return EnumUtils.fromValue(PoleMaxHeight.class, value);
    }
  }

  /** Utility class for working with enums that implement {@link ValueEnum}. */
  public final class EnumUtils {
    private EnumUtils() {
    }

    /**
     * Finds an enum constant from its integer value.
     *
     * @param <T>       the type of the enum
     * @param enumClass the class of the enum
     * @param value     the integer value
     * @return the matching enum constant
     * @throws IllegalArgumentException if no matching enum constant exists
     */
    public static <T extends Enum<T> & ValueEnum> T fromValue(Class<T> enumClass, int value) {
      return Stream.of(enumClass.getEnumConstants())
          .filter(e -> e.getValue() == value)
          .findFirst()
          .orElseThrow(
              () -> new IllegalArgumentException(
                  String.format("No %s with value: %d", enumClass.getSimpleName(), value)));
    }
  }
}
