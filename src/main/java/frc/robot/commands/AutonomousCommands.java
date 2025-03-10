// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Constants;
import frc.robot.state.RobotState;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.outtake.Outtake;
import frc.robot.util.Enums.Height;

/** Add your docs here. */
public class AutonomousCommands {

  private static Command autonomousElevator(Elevator elevator, Height height) {
    double meters = Constants.FieldConstants.heightMap.get(height).doubleValue();
    return new FunctionalCommand(
        () -> {},
        () -> elevator.setHeight(meters),
        interrupted -> {},
        () -> !RobotState.hasObject() || elevator.isAtSetpoint(),
        elevator);
  }

  public static Command autonomousHeight(Elevator elevator, Outtake outtake, Height height) {
    return Commands.sequence(
        BaseCommands.setPivot(outtake, 5),
        autonomousElevator(elevator, height),
        BaseCommands.setPivot(outtake, 0));
  }
}
