// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.outtake.Outtake;
import frc.robot.util.Enums.Height;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HeightSequence extends SequentialCommandGroup {
  /** Creates a new HeightSequence. */
  public HeightSequence(Outtake outtake, Elevator elevator, Height height) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new PivotSet(outtake, 5),
        new ElevatorSet(elevator, Constants.FieldConstants.heightMap.get(height).doubleValue()));
  }
}
