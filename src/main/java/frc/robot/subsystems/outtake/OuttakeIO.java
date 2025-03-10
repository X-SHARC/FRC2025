package frc.robot.subsystems.outtake;

import org.littletonrobotics.junction.AutoLog;

/**
 * Interface for the Outtake subsystem, defining the methods and inputs required for controlling the
 * outtake mechanism.
 */
public interface OuttakeIO {

  @AutoLog
  public static class OuttakeIOInputs {
    // Indicates if the pivot motor is connected
    public boolean pivotMotorConnected = false;
    // Current in Amperes drawn by the pivot motor
    public double pivotMotorCurrentAmps = 0.0;
    // Voltage applied to the pivot motor
    public double pivotMotorAppliedVolts = 0.0;
    // Position of the pivot motor in radians
    public double pivotMotorPositionRad = 0.0;
    // Velocity of the pivot motor in radians per second
    public double pivotMotorVelocityRadPerSec = 0.0;

    // Indicates if the outtake motor is connected
    public boolean outtakeMotorConnected = false;
    // Current in Amperes drawn by the outtake motor
    public double outtakeMotorCurrentAmps = 0.0;
    // Voltage applied to the outtake motor
    public double outtakeMotorAppliedVolts = 0.0;

    // Angle of the pivot in degrees
    public double pivotAngleDegrees = 0.0;
  }

  /**
   * Updates the input values for the outtake subsystem.
   *
   * @param inputs The input values to be updated.
   */
  public default void updateInputs(OuttakeIOInputs inputs) {}

  /**
   * Sets the voltage for the pivot motor.
   *
   * @param voltage The voltage to be applied to the pivot motor.
   */
  public default void setPivotVoltage(double voltage) {}

  /**
   * Sets the voltage for the outtake motor.
   *
   * @param voltage The voltage to be applied to the outtake motor.
   */
  public default void setOuttakeVoltage(double voltage) {}

  public default boolean hasAlgea() {
    return false;
  }

  /**
   * Gets the current position of the pivot motor.
   *
   * @return The position of the pivot motor in radians.
   */
  public default double getPivotPosition() {
    return 0.0;
  }

  /**
   * Gets the current drawn by the pivot motor.
   *
   * @return The current in Amperes drawn by the pivot motor.
   */
  public default double getPivotCurrent() {
    return 0.0;
  }

  /**
   * Gets the velocity of the pivot motor.
   *
   * @return The velocity of the pivot motor in radians per second.
   */
  public default double getPivotVelocity() {
    return 0.0;
  }

  /**
   * Gets the current drawn by the outtake motor.
   *
   * @return The current in Amperes drawn by the outtake motor.
   */
  public default double getOuttakeCurrent() {
    return 0.0;
  }

  /**
   * Checks if the beam break sensor is triggered.
   *
   * @return True if the beam break sensor is triggered, false otherwise.
   */
  public default boolean isBeamBreakTriggered() {
    return false;
  }

  /**
   * Checks if the outtake mechanism is at the setpoint.
   *
   * @return True if the outtake mechanism is at the setpoint, false otherwise.
   */
  public default boolean isAtSetpoint() {
    return false;
  }

  /**
   * Sets the angle of the pivot.
   *
   * @param angle The angle to set the pivot to, in degrees.
   */
  public default void setPivotAngle(double angle) {}

  /** Resets the encoder for the pivot motor. */
  public default void resetPivotEncoder() {}
}
