package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.outtake.Outtake;
import frc.robot.util.Enums.Height;
import java.util.function.BooleanSupplier;

public class BaseCommands {

  private BaseCommands() {}

  public static Command setElevator(Elevator elevator, double meters) {
    return new FunctionalCommand(
        () -> {},
        () -> elevator.setHeight(meters),
        interrupted -> {},
        elevator::isAtSetpoint,
        elevator);
  }

  public static Command setPivot(Outtake outtake, double angle) {
    return new FunctionalCommand(
        () -> {},
        () -> outtake.setPivotAngle(angle),
        interrupted -> {},
        outtake::isAtSetpoint,
        outtake);
  }

  public static Command setHeight(Elevator elevator, Outtake outtake, Height height) {
    double meters = Constants.FieldConstants.heightMap.get(height).doubleValue();
    return new SequentialCommandGroup(setPivot(outtake, 5), setElevator(elevator, meters));
  }

  public static Command homeElevator(Elevator elevator, Outtake outtake) {

    return new SequentialCommandGroup(setPivot(outtake, 5), setElevator(elevator, 0.025));
  }

  public static Command manipulateObject(
      Outtake outtake, double volts, double angle, BooleanSupplier isFinished) {
    return new FunctionalCommand(
        () -> {},
        () -> {
          outtake.setPivotAngle(angle);
          outtake.setOuttakeVoltage(volts);
        },
        interrupted -> outtake.stop(),
        isFinished,
        outtake);
  }

  public static Command intakeCoral(Outtake outtake) {
    return manipulateObject(outtake, 5, 10, outtake::isBeamBreakTriggered);
  }

  public static Command intakeAlgea(Outtake outtake) {
    return manipulateObject(outtake, 5, 10, () -> false);
  }

  public static Command outCoral(Outtake outtake) {
    return manipulateObject(outtake, 5, 0, () -> false);
  }

  public static Command outAlgea(Outtake outtake) {
    return manipulateObject(outtake, -5, 10, () -> false);
  }
}
