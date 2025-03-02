// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.outtake;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.state.RobotState;
import frc.robot.util.Enums.GameObject;
import org.littletonrobotics.junction.Logger;

public class Outtake extends SubsystemBase {
  private final OuttakeIO io;
  private final OuttakeIOInputsAutoLogged inputs;
  private final Alert disconnectedAlerts[];

  /**
   * Creates a new Outtake subsystem.
   *
   * @param io The input/output interface for the outtake subsystem.
   */
  public Outtake(OuttakeIO io) {
    this.io = io;
    this.inputs = new OuttakeIOInputsAutoLogged();
    this.disconnectedAlerts = new Alert[2];
    for (int i = 0; i < disconnectedAlerts.length; i++) {
      disconnectedAlerts[i] = new Alert(
          "Outtake motor " + Integer.toString(i) + " is disconnected.",
          Alert.AlertType.kWarning);
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Outtake", inputs);

    disconnectedAlerts[0].set(!inputs.pivotMotorConnected);
    disconnectedAlerts[1].set(!inputs.outtakeMotorConnected);

    Logger.recordOutput("Outtake/beamBreak", io.isBeamBreakTriggered());

    // TODO: add logic to set with limit switch for algea
    if (io.isBeamBreakTriggered()) {
      RobotState.setGameObject(GameObject.CORAL);
    } else {
      RobotState.setGameObject(GameObject.NONE);
    }
  }

  /**
   * Sets the voltage for the pivot motor.
   *
   * @param voltage The voltage to set for the pivot motor.
   */
  public void setPivotVoltage(double voltage) {
    io.setPivotVoltage(voltage);
  }

  /**
   * Sets the voltage for the outtake motor.
   *
   * @param voltage The voltage to set for the outtake motor.
   */
  public void setOuttakeVoltage(double voltage) {
    io.setOuttakeVoltage(voltage);
  }

  /**
   * Gets the position of the pivot motor.
   *
   * @return The position of the pivot motor.
   */
  public double getPivotPosition() {
    return io.getPivotPosition();
  }

  /**
   * Gets the current of the pivot motor.
   *
   * @return The current of the pivot motor.
   */
  public double getPivotCurrent() {
    return io.getPivotCurrent();
  }

  /**
   * Gets the velocity of the pivot motor.
   *
   * @return The velocity of the pivot motor.
   */
  public double getPivotVelocity() {
    return io.getPivotVelocity();
  }

  /**
   * Gets the current of the outtake motor.
   *
   * @return The current of the outtake motor.
   */
  public double getOuttakeCurrent() {
    return io.getOuttakeCurrent();
  }

  /**
   * Checks if the beam break sensor is triggered.
   *
   * @return True if the beam break sensor is triggered, false otherwise.
   */
  public boolean isBeamBreakTriggered() {
    return io.isBeamBreakTriggered();
  }

  /**
   * Sets the angle of the pivot motor.
   *
   * @param angle The angle to set for the pivot motor.
   */
  public void setPivotAngle(double angle) {
    io.setPivotAngle(angle);
  }

  /**
   * Checks if the pivot motor is at the setpoint.
   *
   * @return True if the pivot motor is at the setpoint, false otherwise.
   */
  public boolean isAtSetpoint() {
    return io.isAtSetpoint();
  }

  /** Resets the pivot encoder. */
  public void resetPivotEncoder() {
    io.resetPivotEncoder();
  }

  /**
   * Stops both the pivot and outtake motors by setting their voltages to zero.
   */
  public void stop() {
    setPivotVoltage(0);
    setOuttakeVoltage(0);
  }
}
