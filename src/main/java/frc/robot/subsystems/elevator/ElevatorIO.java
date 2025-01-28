package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  @AutoLog
  public static class ElevatorIOInputs {
    public boolean masterMotorConnected = false;
    public double masterMotorCurrentAmps = 0.0;
    public double masterMotorAppliedVolts = 0.0;
    public double masterMotorPositionRad = 0.0;
    public double masterMotorVelocityRadPerSec = 0.0;

    public boolean slaveMotorConnected = false;
    public double slaveMotorCurrentAmps = 0.0;
    public double slaveMotorAppliedVolts = 0.0;
    public double slaveMotorPositionRad = 0.0;
    public double slaveMotorVelocityRadPerSec = 0.0;

    public double elevatorCurrentAmps = 0.0;
    public double elevatorPositionMeters = 0.0;
    public double elevatorVelocityMetersPerSec = 0.0;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setVoltage(double voltage) {}

  public default double getPosition() {
    return 0.0;
  }

  public default double getCurrent() {
    return 0.0;
  }

  public default double getVelocity() {
    return 0.0;
  }

  public default void resetEncoders() {}
}
