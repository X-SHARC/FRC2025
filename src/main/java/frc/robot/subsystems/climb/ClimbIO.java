package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

/**
 * Interface for the Climb subsystem, defining the methods and inputs required for controlling the
 * climb mechanism.
 */
public interface ClimbIO {

  @AutoLog
  public static class ClimbIOInputs {
    // Indicates if the climb motor is connected
    public boolean climbMotorConnected = false;
    // Current in Amperes drawn by the climb motor
    public double climbMotorCurrentAmps = 0.0;
    // Voltage applied to the climb motor
    public double climbMotorAppliedVolts = 0.0;
    // Position of the climb motor in radians
    public double climbMotorPositionRad = 0.0;
    // Velocity of the climb motor in radians per second
    public double climbMotorVelocityRadPerSec = 0.0;
  }

  /**
   * Updates the input values for the climb subsystem.
   *
   * @param inputs The input values to be updated.
   */
  public default void updateInputs(ClimbIOInputs inputs) {}

  /**
   * Sets the voltage for the climb motor.
   *
   * @param voltage The voltage to be applied to the climb motor.
   */
  public default void setClimbVoltage(double voltage) {}

  /**
   * Gets the current position of the climb motor.
   *
   * @return The current position of the climb motor in radians.
   */
  public default double getClimbPosition() {
    return 0.0;
  }

  /**
   * Gets the current velocity of the climb motor.
   *
   * @return The current velocity of the climb motor in radians per second.
   */
  public default double getClimbVelocity() {
    return 0.0;
  }

  /**
   * Gets the current current of the climb motor.
   *
   * @return The current current of the climb motor in amperes.
   */
  public default double getClimbCurrent() {
    return 0.0;
  }

  /**
   * Gets the current voltage of the climb motor.
   *
   * @return The current voltage of the climb motor in volts.
   */
  public default double getClimbVoltage() {
    return 0.0;
  }
}
