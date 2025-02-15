package frc.robot.subsystems.outtake;

import org.littletonrobotics.junction.AutoLog;

public interface OuttakeIO {

  @AutoLog
  public static class OuttakeIOInputs {
    public boolean pivotMotorConnected = false;
    public double pivotMotorCurrentAmps = 0.0;
    public double pivotMotorAppliedVolts = 0.0;
    public double pivotMotorPositionRad = 0.0;
    public double pivotMotorVelocityRadPerSec = 0.0;

    public boolean outtakeMotorConnected = false;
    public double outtakeMotorCurrentAmps = 0.0;
    public double outtakeMotorAppliedVolts = 0.0;

    public double pivotAngleDegrees = 0.0;
  }

  public default void updateInputs(OuttakeIOInputs inputs) {}

  public default void setPivotVoltage(double voltage) {}

  public default void setOuttakeVoltage(double voltage) {}

  public default double getPivotPosition() {
    return 0.0;
  }

  public default double getPivotCurrent() {
    return 0.0;
  }

  public default double getPivotVelocity() {
    return 0.0;
  }

  public default double getOuttakeCurrent() {
    return 0.0;
  }

  public default boolean isBeamBreakTriggered() {
    return false;
  }

  public default boolean isAtSetpoint() {
    return false;
  }

  public default void setPivotAngle(double angle) {}

  public default void resetPivotEncoder() {}
}
