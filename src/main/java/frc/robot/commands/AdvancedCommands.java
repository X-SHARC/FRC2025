// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.state.RobotState;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.outtake.Outtake;
import frc.robot.util.Enums.Height;

/** Add your docs here. */
public class AdvancedCommands {
    public static Command autoElevatorAndManipulate(Elevator elevator, Outtake outtake) {

        Height h = RobotState.getSelectedElevatorHeight();

        if (h.equals(Height.ZERO)) {
            return BaseCommands.homeElevator(elevator, outtake);
        }

        Command intakeCommand = h.equals(Height.ALGEA_LOW) || h.equals(Height.ALGEA_HIGH)
                ? BaseCommands.intakeAlgea(outtake)
                : BaseCommands.intakeCoral(outtake);
        return Commands.sequence(BaseCommands.setHeight(elevator, outtake, h), intakeCommand);
    }
}
