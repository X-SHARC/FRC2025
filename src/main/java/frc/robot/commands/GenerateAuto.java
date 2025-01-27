// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SharcGen;
import frc.robot.state.RobotState;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.util.Enums.GameObject;
import frc.robot.util.Enums.Position;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GenerateAuto extends Command {
  Elevator elevator;
  SharcGen gen;

  int side;
  Position pos;
  boolean hasEnded = false;

  SequentialCommandGroup seq;

  /** Creates a new GenerateAuto. */
  public GenerateAuto(Elevator elevator) {
    this.elevator = elevator;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    gen = new SharcGen();
    seq = gen.generateHalfCoralCycle(elevator);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (seq.isFinished() && RobotState.isAuto()) {
      flip();
      seq = gen.generateHalfCoralCycle(elevator);
    }

    if (RobotState.isAuto()) {
      seq.execute();
    } else {
      seq.end(true);
    }
  }

  private void flip() {
    if (RobotState.hasObject()) {
      RobotState.setGameObject(GameObject.NONE);
    } else {
      RobotState.setGameObject(GameObject.CORAL);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (seq != null && !seq.isFinished()) {
      seq.end(interrupted);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !RobotState.isAuto();
  }
}
