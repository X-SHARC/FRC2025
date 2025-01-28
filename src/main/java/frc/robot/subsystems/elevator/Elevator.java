// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs;
  private final Alert disconnectedAlerts[];

  /** Creates a new Elevator. */
  public Elevator(ElevatorIO io) {
    this.io = io;
    this.inputs = new ElevatorIOInputsAutoLogged();
    this.disconnectedAlerts = new Alert[2];
    for (int i = 0; i < disconnectedAlerts.length; i++) {
      disconnectedAlerts[i] = new Alert(
          "Elevator motor " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    disconnectedAlerts[0].set(!inputs.masterMotorConnected);
    disconnectedAlerts[1].set(!inputs.slaveMotorConnected);
  }

  @Override
  public void simulationPeriodic() {
    Logger.recordOutput(
        "Elevator/Pose/FirstPose3D", new Pose3d(0, 0, getPosition(), new Rotation3d()));
    Logger.recordOutput(
        "Elevator/Pose/CarriagePose3D",
        new Pose3d(0, 0, getPosition() * 1.8, new Rotation3d()));
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  public void setPercent(double percent) {
    io.setVoltage(percent * 12);
  }

  public double getPosition() {
    return io.getPosition();
  }

  public double getVelocity() {
    return io.getVelocity();
  }

  public double getCurrent() {
    return io.getCurrent();
  }

  public void stop() {
    setVoltage(0);
  }

  public boolean isAtSetpoint() {
    return io.isAtSetpoint();
  }

  public void setHeight(double height) {
    io.setHeight(height);
  }

  public void resetEncoders() {
    io.resetEncoders();
  }
}
