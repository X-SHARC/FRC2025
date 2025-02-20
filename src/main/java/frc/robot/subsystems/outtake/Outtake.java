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

  /** Creates a new Outtake. */
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

  public void setPivotVoltage(double voltage) {
    io.setPivotVoltage(voltage);
  }

  public void setOuttakeVoltage(double voltage) {
    io.setOuttakeVoltage(voltage);
  }

  public double getPivotPosition() {
    return io.getPivotPosition();
  }

  public double getPivotCurrent() {
    return io.getPivotCurrent();
  }

  public double getPivotVelocity() {
    return io.getPivotVelocity();
  }

  public double getOuttakeCurrent() {
    return io.getOuttakeCurrent();
  }

  public boolean isBeamBreakTriggered() {
    return io.isBeamBreakTriggered();
  }

  public void setPivotAngle(double angle) {
    io.setPivotAngle(angle);
  }

  public boolean isAtSetpoint() {
    return io.isAtSetpoint();
  }

  public void resetPivotEncoder() {
    io.resetPivotEncoder();
  }

  public void stop() {
    setPivotVoltage(0);
    setOuttakeVoltage(0);
  }
}
