// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.state.RobotState;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.util.Enums.Height;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HomeElevator extends SequentialCommandGroup {
  /** Creates a new HomeElevator. */
  public HomeElevator(Elevator elevator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ParallelDeadlineGroup(
            new WaitCommand(0.05), new RunCommand(() -> elevator.setPercent(-0.1), elevator)),
        new RunCommand(() -> elevator.setPercent(-0.1), elevator)
            .until(() -> elevator.getCurrent() > ElevatorConstants.HomeCurrent),
        new InstantCommand(() -> elevator.setPercent(0.0), elevator),
        new InstantCommand(() -> elevator.resetEncoders(), elevator),
        new InstantCommand(() -> RobotState.setElevatorHeight(Height.ZERO), elevator));
  }
}
